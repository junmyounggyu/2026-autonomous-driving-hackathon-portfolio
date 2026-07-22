# CARLA-ROS2 Autonomous Racing: Stanley + PID Path Tracking Controller

성균관대학교 기계공학부 전공 교과 「모바일시스템제어(EME3081)」 텀 프로젝트로, ROS2와 CARLA 시뮬레이터를 이용해 주어진 트랙을 최단 시간에 완주하는 자율주행 제어 알고리즘을 설계·구현했습니다.

- 기간: 2025.12 (텀 프로젝트)
- 소속: 성균관대학교 기계공학부, Group 1
- 담당 역할: Stanley + PID 주행 제어 코드, 경로 최적화 코드 구현
- 결과: 최적 경로 이탈 없이 완주 성공, 최종 랩타임 113초대 기록

## 1. 개발 환경

| 항목 | 내용 |
|---|---|
| OS | Ubuntu 20.04 (듀얼 부팅) |
| 미들웨어 | ROS2 Foxy |
| 시뮬레이터 | CARLA |
| 언어/라이브러리 | Python 3, rclpy, NumPy, Pandas |
| 경로 최적화 참조 오픈소스 | [TUMFTM/global_racetrajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization) |

차량과 시뮬레이터 간 통신은 ROS2 토픽 기반으로 이루어집니다.

- Subscribe: `/mobile_system_control/ego_vehicle` (`std_msgs/Float32MultiArray`) — 차량의 x, y, heading, velocity, steering angle
- Publish: `/mobile_system_control/control_msg` (`geometry_msgs/Vector3Stamped`) — throttle(x, 0~1), steer(y, -1~1), brake(z, 0~1)

## 2. 이론적 배경

### Stanley Control (횡방향 제어)
전륜 차축 중심을 기준으로, 헤딩 오차와 횡방향 오차(cross-track error)를 동시에 보정하여 조향각을 계산합니다.

- 헤딩 오차: 경로 접선 방향과 차량 진행 방향의 차이
- 횡방향 오차: 전륜 중심에서 경로까지의 최단 거리
- 조향각: `δ(t) = ψ_e(t) + arctan(k·e(t) / v(t))`

Pure Pursuit 대비 헤딩 오차까지 함께 보정하므로 급코너에서 정밀한 추종이 가능하고, LQR/MPC 대비 복잡한 차량 모델 없이 게인 튜닝만으로 가벼운 연산의 실시간 제어가 가능합니다.

### PID + Feedforward (종방향 제어)
시뮬레이터는 속도/가속도를 직접 입력받지 않고 throttle·brake를 입력받는 구조이므로, 목표 속도와의 오차를 throttle/brake로 변환하는 PID 제어를 사용했습니다.

- 비례(P): 응답 속도 향상
- 적분(I): 정상상태 오차(steady-state error) 제거
- 미분(D): 급가감속 시 오버슈트·진동 억제
- Feedforward(FF): 최적 경로 생성 시 계산된 목표 가속도(target_a)를 미리 반영해, 감속·가속 명령의 지연을 줄임

`u_k = Kp·e_k + Ki·Σ(e_j·Δt) + Kd·(e_k - e_k-1)/Δt + Kf·a_target`

## 3. 구현, 검증, 개선 과정

프로젝트는 4단계로 진행했습니다.

**① Stanley + P (트랙 중간점 추종)**
통신 및 주행 코드 정상 동작 여부를 검증하기 위한 초기 버전. 트랙 중간점(waypoints.csv)을 단순 추종하며, 종방향은 P 제어만 사용. 연습용 맵에서는 성공했으나 본선 맵에서는 목표 속도 추종 오차와 코너 진입 시 트랙 이탈이 발생.

**② Stanley + PID + Feedforward로 개선**
고속 주행 시 발생한 정상상태 속도 오차를 적분 제어로 보정하고, 미분 제어로 속도 변화 응답을 부드럽게 만들었습니다. 추가로 목표 가속도 기반 피드포워드를 넣어 급가속/급감속 구간의 응답 지연을 개선했습니다.

**③ 경로 최적화 코드 구현**
트랙 중심선 좌표를 (s, n) 좌표계로 변환해, 곡률을 최소화하는 방향으로 각 지점의 횡방향 오프셋(n)을 최적화했습니다. 목적함수는 랩타임 `Minimize J = Σ(Δs/v)`이며, 차량 폭과 안전 여유를 고려한 트랙 폭 제약, 타이어 마찰력 기반 가감속 제약(`a_long² + a_lat² ≤ (μg)²`)을 반영했습니다. TUM FTM의 오픈소스를 참조해 위치·속도·헤딩·가속도 프로파일이 담긴 최적 경로(opt_trj.csv)를 생성했습니다.

**④ 게인 튜닝 및 최종 코드 완성**
Stanley gain, PID의 Kp/Ki/Kd, feedforward gain(kf) 총 4개 게인을 튜닝했습니다. 시간 제약상 kf 위주로 튜닝을 진행했고, kf=0.1일 때 랩타임과 주행 안정성의 균형이 가장 좋았습니다(kf=0 → 115.68s, kf=0.1 → 113.64s, kf=0.15 → 114.88s, kf=0.25 → 요철 구간 진동 심화).

또한 실행 구조를 비동기 타이머 방식에서 센서 데이터 수신 즉시 제어 명령을 계산·발행하는 이벤트 구동(event-driven) 방식으로 변경해 제어 지연시간을 단축했습니다.

## 4. 최종 구현 결과

- 최적 경로 생성: 곡률이 큰 코너 구간에서 out-in-out 궤적이 형성되고, 완만한 구간에서는 직선에 가까운 경로가 생성됨을 확인
- 검증: 최적 경로의 속도·위치 프로파일과 실제 CARLA에서 수신한 ego_vehicle 토픽 데이터를 비교해 추종 성능을 확인
- 랩타임: 1차 주행 113.24s, 2차 주행 113.64s
- 안전성 우선 설계: 트랙 요철 구간에서의 진동·이탈 위험을 고려해 곡률 계산 시 안전 여유폭을 차량 폭 대비 약 34%(2.4m)로 보수적으로 설정, 공격적인 기록 단축보다 완주 안정성을 우선함
- 한계 및 개선 방향: 현재 제어기는 현재 시점 오차와 전방 일부 곡률만 고려하므로 연속 코너 구간에서 불필요한 감속·오버슈트가 발생할 수 있음. 향후 차량 동역학 모델 기반으로 미래 거동을 예측하는 MPC(Model Predictive Control) 도입 시 조향·가감속을 하나의 최적화 문제로 통합해 고속 주행 안정성을 개선할 수 있을 것으로 판단

## 5. 파일 구성

- `racing_node.py` — 최적 경로(opt_trj.csv)를 추종하는 Stanley(횡방향) + PID+FF(종방향) 최종 주행 제어 코드
- `Final_Report.pdf` — 최종 보고서 (수식, 그래프, 전체 Appendix 코드 포함)

## Reference

- J.-T. Li, C.-K. Chen, H. Ren, "Time-Optimal Trajectory Planning and Tracking for Autonomous Vehicles," Sensors, vol. 24, no. 11, 2024.
- TUMFTM, "global_racetrajectory_optimization," GitHub. https://github.com/TUMFTM/global_racetrajectory_optimization
