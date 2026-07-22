# AI Chip 자율주행 소형차: PYNQ-DPU 차선 인식 주행 & 초음파 직각 주차

성균관대학교 AI 반도체(AI Chip) 설계 교과/대회(팀명: skkubullracing)에서 진행한 임베디드 자율주행 소형차 프로젝트입니다. Xilinx PYNQ 보드에 탑재된 DPU(Deep Learning Processor Unit)로 YOLO 기반 차선 인식을 가속하여 주행하는 파트와, 카메라 없이 초음파 센서만으로 직각 주차를 수행하는 파트로 구성되어 있습니다. 본인은 이 중 **주행(P_driving) 파트**를 담당했고, **주차(parking) 파트**의 알고리즘 설계에 참여했습니다.

- 소속: 성균관대학교, 팀 skkubullracing
- 저장소: [ai-chip-skkubullracing-pynq](https://github.com/LeeTaeHyeon038/ai-chip-skkubullracing-pynq)
- 담당 역할: 차선 인식 기반 자율주행 제어(`P_driving`), 초음파 센서 기반 직각 주차 알고리즘 설계(`parking`)

## 1. 개발 환경

| 항목 | 내용 |
|---|---|
| 보드 | Xilinx PYNQ 기반 임베디드 보드 (Ultra96-V2) |
| 가속기 | Xilinx DPU (Deep Learning Processor Unit) — `dpu.bit` 오버레이 |
| 신경망 모델 | Tiny-YOLOv3 (256×256, 차선 검출용 xmodel) |
| 언어/프레임워크 | Python 3, PYNQ (`pynq`, `pynq_dpu`), OpenCV, NumPy, spidev, keyboard |
| 개발/실행 방식 | PYNQ 보드 내 Jupyter Lab(`http://<IP>:9090/lab`)에서 개발 후, 대회 당일은 `.py`로 변환하여 실행 |
| 통신 | USB-C(데이터 전용) 연결 후 Jupyter 접속 — JTAG/UART는 리셋 신호가 물려 있어 대회 중 케이블 분리 시 프로세스가 죽으므로 디버깅 전용으로만 사용 |
| 조향 각도 측정 | 핸들 축 가변저항(ADC) → SPI로 읽음 |
| 조향/구동 액추에이터 | 모터 6개(좌우 후륜 각 2개 + 조향 좌/우) — MMIO로 duty 제어 |
| 주차 센서 | 초음파 거리 센서 5개 (원래 가이드 12개 중 5개만 지급) |

## 2. 이론적 배경

### 2-1. 차선 인식 기반 조향 제어 (P_driving)

카메라 프레임을 BEV(Bird's Eye View, 조감도)로 원근 변환한 뒤, DPU에 올린 경량 YOLO(tiny-yolov3, 256×256)로 차선 박스를 검출합니다. 검출된 박스 중 화면에서 가장 오른쪽에 있는 차선의 중심 x좌표를 구하고, 기준점(`REFERENCE_X`, `REFERENCE_Y`)과 이 중심점을 잇는 직선의 기울기로 목표 조향각을 계산합니다.

```
slope = (x2 - x1) / (y1 - y2)
angle = atan(slope)  [deg]
```

계산된 각도는 그대로 쓰지 않고 다음 단계를 거쳐 조향 목표값(-7~7)으로 변환합니다.

1. **데드존**: 각도가 2도 미만이면 0으로 처리해 미세한 흔들림 무시
2. **저역통과필터(LPF)**: `target_slope_f = (1-α)·이전값 + α·현재값`으로 급격한 값 변화 완화
3. **스케일링**: 카메라 최대 각도(`THETA_MAX_DEG`) 기준으로 -7~7 범위에 매핑
4. **변화율 제한**: 프레임당 목표값 변화폭을 `MAX_STEP_DELTA`로 제한해 급조향 방지

이렇게 만든 목표 조향값과 가변저항(ADC)으로 읽은 실제 조향각의 오차에 **비례(P) 제어**를 적용해 조향 모터 duty를 결정합니다.

```
error = target - 실측 조향각
u = Kp × error
duty = clip(|u|, DUTY_MIN, DUTY_MAX)
```

오차가 데드밴드 이내면 조향 모터를 정지시켜 헌팅(hunting)을 방지했습니다. 차선이 일정 프레임(`LOST_LANE_THRESHOLD`) 이상 미검출되면 직전 유효 각도를 유지하다가, 임계를 넘으면 강제 선회각으로 전환해 트랙 이탈을 방지했습니다. 또한 차선이 `LAUNCH_SUCCESS_FRAMES` 프레임 연속으로 안정적으로 인식된 뒤에만 후륜을 출발시키는 출발 게이트를 두어, 초기 오검출로 인한 오발진을 막았습니다.

### 2-2. 초음파 센서 기반 직각 주차 (parking)

카메라 없이 초음파 센서 5개(원 가이드 12개 중 선정)만으로 두 장애물 차량 사이에 후진 직각 주차를 수행합니다. 절대 거리값의 신뢰도가 낮으므로, 원시 tick count를 거리(cm)로 변환한 뒤 **중앙값(median) 필터**(창 크기 5)로 튀는 값만 걸러내고, 상태 전환 판정에는 거리를 5단계 밴드(아주가까움~아주멈)로 양자화해 **히스테리시스**를 적용했습니다.

주차는 센서 이벤트 기반의 상태 머신(state machine)으로 구성했습니다.

```
SEARCH_GAP → SWING_LEFT → REVERSE_RIGHT → ALIGN_REVERSE → HOLD
→ EXIT_FORWARD → EXIT_TURN → EXIT_STRAIGHT → DONE
```

- `SEARCH_GAP`: 중립 조향으로 전진하며 우측 전·후측면 센서(S2, S0)가 동시에 빈 공간을 가리키는 순간 정지 (빈 주차 공간 탐색)
- `SWING_LEFT` → `REVERSE_RIGHT`: 최대 좌조향 전진 후 최대 우조향 후진으로 주차 공간에 진입
- `ALIGN_REVERSE`: 우측 전·후측면 센서 차이(S0-S2)로 차체 기울기를 계산해 조향을 미세 보정하며 평행하게 저속 후진, 후방 45도 코너 센서(S9, S11)가 양옆 차량을 더 이상 감지하지 않는 시점을 후진 종료 신호로 사용
- `HOLD` → `EXIT_*`: 규정상 요구되는 3~5초 정차 후 직진·회전으로 출차

각 상태의 종료 조건은 "감지 → 해제" 또는 "해제 → 감지"의 **2단계 래치**로 설계했습니다. 상태 진입 직후 중앙값 필터가 이전 구간의 잔상값을 들고 있어 조건이 즉시 만족되는 문제가 있었는데, 순서를 요구하는 래치를 두어 이를 방지했습니다. 대회 규정상 좌우 바퀴 차동 회전이 금지되어 있어 방향 전환은 오직 조향 모터로만 수행하도록 설계했고, 출발·장애물 위치가 대회 당일 추첨으로 정해지고 이후 코드 수정이 불가능했기 때문에 거리·시간을 하드코딩하지 않고 모든 상태 전환을 센서 이벤트 기반으로 구현했습니다(단, 초음파로 측정 불가능한 90도 회전과 각 상태의 타임아웃 안전장치만 예외적으로 시간 기반 사용).

## 3. 구현 및 검증

### 3-1. 주행 (`P_driving/`)

| 파일 | 역할 |
|---|---|
| `main.py` | 대회용 실행 진입점. DPU 오버레이·xmodel 로드 후 컨트롤러 실행 |
| `image_processor.py` | BEV 변환 → DPU 추론(YOLO) → 차선 중심각 계산 |
| `motor_controller.py` | 조향 P제어(LPF·데드존·변화율 제한 포함), 후륜 구동, SPI ADC 읽기, 출발 게이트 |
| `driving_system_controller.py` | 자율/수동 모드 전환, Space 시작·정지, 프레임 루프 |
| `config.py` | 조향 게인(Kp), duty 상하한, LPF 계수, 가변저항 실측값 등 튜닝 파라미터 |
| `yolo_utils.py` | Tiny-YOLOv3 후처리(NMS, 박스 디코딩) |

원본 레포의 기본 골격(수동 모드 W/A/S/D/R, 모드 전환 1/2, Space 시작·정지, Q 종료)은 그대로 유지하면서, 자율주행 조향 로직만 3단계 on/off 방식에서 연속 P제어로 교체했습니다. 대회 당일에는 디버깅 PC를 분리해야 하므로, 리셋 신호가 함께 물려 있는 JTAG/UART 대신 단순 데이터 케이블인 USB-C로 PYNQ의 Jupyter에 접속해 실행하도록 운영 절차를 확정했고, 이를 사전에 실측으로 검증했습니다.

### 3-2. 주차 (`parking/`)

`parking_debug_v2.ipynb`에서 상태 머신, 센서 필터링(median filter), 밴드·히스테리시스 판정 로직을 개발·디버깅했습니다. 원본 가이드라인의 센서 12개·7단계 알고리즘을 5개 센서 환경에 맞게 재설계하는 과정에서, 다음과 같은 설계 판단을 문서화(`초음파_센서_기반_자율주행_직각_주차_가이드라인.md`)했습니다.

- 5개 센서 배치(S0, S2, S8, S9, S11) 선정 근거와 각 센서의 부착 각도(특히 S9·S11의 후방 바깥쪽 45도) 결정
- 원본 6단계(좌우 중앙 정렬)는 비홀로노믹 차량 특성상 각도·위치 두 목표를 동시에 만족시킬 수 없어 제거하고, 각도(평행) 정렬만 수행하도록 단순화
- 이동평균/EMA 대신 경계를 보존하는 중앙값 필터 채택 근거
- 규정에서 파생된 추가 요구사항(출차 단계, 차동 회전 금지, 하드코딩 금지, 충돌 시 즉시 정지) 반영

## 4. 파일 구성 (원본 저장소 기준)

- [`P_driving/`](https://github.com/LeeTaeHyeon038/ai-chip-skkubullracing-pynq/tree/main/P_driving) — DPU 기반 차선 인식 자율주행 제어 코드 (`main.py`, `image_processor.py`, `motor_controller.py`, `driving_system_controller.py`, `config.py`, `yolo_utils.py`)
- [`parking/`](https://github.com/LeeTaeHyeon038/ai-chip-skkubullracing-pynq/tree/main/parking) — 초음파 센서 기반 직각 주차 알고리즘 (`parking_debug_v2.ipynb`, 설계 가이드라인 문서)
