import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
import matplotlib.pyplot as plt

class SimpleKalmanFilter:
    def __init__(self, init_X, init_P, init_Q):
        assert isinstance(init_X, list) and len(init_X) == 2
        assert isinstance(init_P, list) and len(init_P) == 2
        assert isinstance(init_Q, list) and len(init_Q) == 2

        self.X = np.array(init_X).reshape(2, 1)
        self.P = np.diag(init_P)
        
        # --- [TODO 채워진 부분] 관측 행렬 (H) ---
        # 측정값이 위치(x) 하나뿐이므로, 상태 행렬 [x, vx]에서 x만 뽑아오도록 [1, 0]으로 설정합니다.
        self.H = np.array([
            [1.0, 0.0]
        ])
        # ----------------------------------------
        
        self.Q = np.diag(init_Q)

        self.dt = 0.1

        # --- [TODO 채워진 부분] 상태 변환 행렬 (A) ---
        # 다음 시점의 x = 현재 x + (dt * 현재 vx)
        # 다음 시점의 vx = 현재 vx (등속도 모델 가정)
        self.A = np.array([
            [1.0, self.dt],
            [0.0, 1.0]
        ])
        # ----------------------------------------

    def predict_step(self):
        # --- [TODO 채워진 부분] 예측 스텝 ---
        # 1. 상태 예측: X = A * X
        self.X = self.A @ self.X
        # 2. 오차 공분산 예측: P = A * P * A^T + Q
        self.P = self.A @ self.P @ self.A.T + self.Q
        # ----------------------------------------

    def update_step(self, Z, R):
        assert isinstance(Z, list) and len(Z) == 1
        assert isinstance(R, list) and len(R) == 1

        Z_vec = np.array(Z).reshape(1, 1)
        R_mat = np.diag(R)
        
        # --- [TODO 채워진 부분] 업데이트 스텝 ---
        # 1. 측정 잔차 (Y = Z - H * X)
        Y = Z_vec - (self.H @ self.X)
        
        # 2. 잔차 공분산 (S = H * P * H^T + R)
        S = (self.H @ self.P @ self.H.T) + R_mat
        
        # 3. 칼만 이득 (K = P * H^T * S^-1)
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # 4. 상태 업데이트 (X = X + K * Y)
        self.X = self.X + (K @ Y)
        
        # 5. 오차 공분산 업데이트 (P = (I - K * H) * P)
        I = np.eye(self.P.shape[0])
        self.P = (I - (K @ self.H)) @ self.P
        # ----------------------------------------


class KalmanFilter:
    def __init__(self, init_X, init_P, init_Q):
        assert isinstance(init_X, list) and len(init_X) == 6
        assert isinstance(init_P, list) and len(init_P) == 6
        assert isinstance(init_Q, list) and len(init_Q) == 6

        self.X = np.array(init_X).reshape(6, 1) # [x, y, theta, vx, vy, v_theta]
        self.P = np.diag(init_P)
        
        # --- [TODO 채워진 부분] 관측 행렬 (H) ---
        # 6개의 상태 중 위치(x, y)와 헤딩(theta) 3개만 측정하므로 3x6 행렬이 됩니다.
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        ])
        # ----------------------------------------
        
        self.Q = np.diag(init_Q)

        self.dt = 0.1

        # --- [TODO 채워진 부분] 상태 변환 행렬 (A) ---
        # 등속도(CV) 모델을 3차원(x, y, theta)으로 확장한 행렬입니다.
        self.A = np.array([
            [1.0, 0.0, 0.0, self.dt, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, self.dt, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, self.dt],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        ])
        # ----------------------------------------
    
    @staticmethod
    def angle_mod(x):
        return x % (2 * np.pi)
    
    @staticmethod
    def residual_mod(x):
        return (x + np.pi / 2) % np.pi - np.pi / 2

    def predict_step(self):
        # --- [TODO 채워진 부분] 예측 스텝 ---
        self.X = self.A @ self.X
        self.P = self.A @ self.P @ self.A.T + self.Q
        # ----------------------------------------
        self.X[2, 0] = self.angle_mod(self.X[2, 0]) # 예측 값의 헤딩 범위 조정

    def update_step(self, Z, R):
        assert isinstance(Z, list) and len(Z) == 3
        assert isinstance(R, list) and len(R) == 3

        # [수정] Z가 길이 3이므로 3x1로 reshape 해야 수학적으로 맞습니다.
        Z_vec = np.array(Z).reshape(3, 1) 
        R_mat = np.diag(R)

        # --- [TODO 채워진 부분] 업데이트 스텝 ---
        Y = Z_vec - (self.H @ self.X)
        
        # Y[2, 0] = self.residual_mod(Y[2, 0]) --> 잔차의 각도 항목에 대해서 범위 조정
        Y[2, 0] = self.residual_mod(Y[2, 0])
        
        S = (self.H @ self.P @ self.H.T) + R_mat
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        self.X = self.X + (K @ Y)
        self.X[2, 0] = self.angle_mod(self.X[2, 0]) # 각도 정규화
        
        I = np.eye(self.P.shape[0])
        self.P = (I - (K @ self.H)) @ self.P
        # ----------------------------------------


def main():
    def get_gt_state(t):
        return 5.0 * np.sin(t), 5.0 * np.cos(t) # x, vx
    
    ##########################
    init_X = [0.0, 0.0]
    init_P = [5.0, 20.0]
    init_Q = [0.1, 1.2]
    init_R = [0.5 ** 2]
    ##########################

    kf = SimpleKalmanFilter(init_X, init_P, init_Q)

    gt_xs = []
    gt_vxs = []
    pred_xs = []
    pred_vxs = []
    
    for i in range(100):
        t = i * 0.1

        gt_x, gt_vx = get_gt_state(t)
        gt_xs.append(gt_x)
        gt_vxs.append(gt_vx)

        # 실제로는 센서 노이즈가 낀 위치(meas_x)만 측정되었다고 가정
        meas_x = np.random.normal(gt_x, 0.5)

        kf.predict_step()
        kf.update_step([meas_x], init_R)
        
        pred_xs.append(kf.X[0, 0])
        pred_vxs.append(kf.X[1, 0])

    plt.figure()
    plt.subplot(2, 1, 1)
    plt.title("kalman filter result")
    plt.plot(gt_xs, label="GT x (Ground Truth)")
    plt.plot(pred_xs, label="PR x (Predicted)")
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(gt_vxs, label="GT vx (Ground Truth)")
    plt.plot(pred_vxs, label="PR vx (Predicted)")
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__=="__main__":
    main()