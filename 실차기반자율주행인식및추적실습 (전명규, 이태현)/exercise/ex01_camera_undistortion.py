import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import cv2
import numpy as np


def main(manual_undistortion):
    """
    Args:
        manual_undistortion : 수동 왜곡 보정 여부 플래그
    """
    # 이미지 로드
    img_path   = "./data/unrectified/image_02/0000000000.png"

    assert os.path.exists(img_path), "image file not found"

    img = cv2.imread(img_path)
    h, w = img.shape[:2]

    # 카메라 내부 파라미터와 왜곡 계수 읽어오기
    calib_path = "./data/unrectified/calib_cam_to_cam.txt"

    assert os.path.exists(calib_path), "calibration file not found"

    def load_K_D_cam_to_cam(calib_path):
        """
        Args:
            calib_path : 캘리브레이션 파일 경로

        Return:
            K : 카메라 내부 파라미터 행렬
            D : 왜곡 계수
        """
        K, D = None, None
        cam_id="02"

        with open(calib_path, "r") as f:
            for line in f:
                if line.startswith(f"K_{cam_id}:"):
                    vals = list(map(float, line.strip().split()[1:]))
                    K = np.array(vals).reshape(3, 3)

                elif line.startswith(f"D_{cam_id}:"):
                    vals = list(map(float, line.strip().split()[1:]))
                    D = np.array(vals, dtype=np.float32)

        if K is None or D is None:
            raise RuntimeError("Failed to load K or D from calib file")

        return K.astype(np.float32), D

    K, D = load_K_D_cam_to_cam(calib_path)

    # print("K:\n", K)
    # print("D:", D)

    if manual_undistortion:
        undistorted = np.zeros_like(img)

        # 결과 이미지(Undistorted)의 모든 픽셀을 순회 (Backward Mapping 시작)
        for v in range(h):
            for u in range(w):
                # 픽셀 좌표를 왜곡이 없는 '이상적인' 정규 좌표계(Normalized Plane)로 변환
                # u, v -> x_u, y_u (중심점 0.5 보정 포함)
                p = np.array([u + 0.5, v + 0.5, 1.0], dtype=np.float32)
                x_u, y_u, _ = np.linalg.inv(K) @ p

                # 렌즈 왜곡 모델 적용 (Brown-Conrady 모델)
                # 정규 좌표(x_u, y_u)를 인위적으로 왜곡시켜 원본 이미지에서의 위치(x_d, y_d)를 찾음
                k1, k2, p1, p2, k3 = D
                r2 = x_u*x_u + y_u*y_u
                r4 = r2*r2
                r6 = r4*r2

                # 방사 왜곡(Radial Distortion) 계산
                radial = 1 + k1*r2 + k2*r4 + k3*r6

                # 접선 왜곡(Tangential Distortion)을 포함한 최종 왜곡 좌표 계산
                x_d = x_u * radial + 2*p1*x_u*y_u + p2*(r2 + 2*x_u*x_u)
                y_d = y_u * radial + p1*(r2 + 2*y_u*y_u) + 2*p2*x_u*y_u

                # 왜곡된 정규 좌표를 다시 원본 이미지의 픽셀 좌표(u_src, v_src)로 변환
                p_src = K @ np.array([x_d, y_d, 1.0], dtype=np.float32)
                u_src = int(round(p_src[0] / p_src[2] - 0.5))
                v_src = int(round(p_src[1] / p_src[2] - 0.5))

                # 원본 이미지의 범위 내에 있다면, 해당 위치의 색상 값을 가져와 결과물에 채움
                # 이 과정을 통해 휘어 있던 원본의 픽셀들이 보정된 위치로 재배치됨
                if 0 <= u_src < w and 0 <= v_src < h:
                    undistorted[v, u] = img[v_src, u_src]

    else:
        # opencv를 이용한 왜곡 보정
        undistorted = cv2.undistort(img, K, D)

    # 결과물 시각화
    combined = np.vstack((img, undistorted))
    cv2.imwrite("./results/undistorted_0000.png", combined)

    cv2.imshow("Raw (Top) / Undistorted (Bottom)", combined)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # 왜곡 전후의 차이 시각화
    diff = cv2.absdiff(img, undistorted)
    diff_gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)

    print("max pixel diff:", diff_gray.max())

    cv2.imwrite("./results/diff_map_0000.png", diff_gray)

    cv2.imshow("pixel difference", diff_gray)
    cv2.waitKey(0)

if __name__ == '__main__':
    main(manual_undistortion=True)
