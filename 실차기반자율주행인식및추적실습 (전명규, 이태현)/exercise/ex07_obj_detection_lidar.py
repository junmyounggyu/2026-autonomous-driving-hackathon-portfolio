import os
import sys
from pathlib import Path

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
import open3d as o3d

import model.conf.kitti_config as cnf
from model.utils.complexYOLO_utils import ComplexYOLO, inference


def inferenceComplexYOLO(model, lidar_points):
    """
    Complex-YOLO 모델을 사용하여 LiDAR 데이터로부터 객체를 검출하고 실제 좌표계로 변환합니다.

    Args:
        model (ComplexYOLO): 사전 학습된 Complex-YOLO 모델 객체.
        lidar_points (np.ndarray): (N, 4) 형태의 LiDAR 포인트 클라우드 (x, y, z, intensity).

    Returns:
        pred_list (list): 검출된 각 객체의 정보를 담은 리스트.
                          포맷: [[x, y, z, w, h, l, yaw, score, type_str], ...]
    """
    preds_raw = inference(model, lidar_points)[0].numpy()

    pred_list = []
    for pred in preds_raw:
        """
        pred는 (x, y, w, l, im, re, object_conf, class_score, class_pred)의 정보를 담고 있음
        팍샐 좌표계의 출력값을 라이다 좌표계로 변환하고, 필요한 정보를 원하는 포맷으로 저장
        [x, y, z, w, h, l, yaw, score, type_str] 형태로 pred_list에 추가
        """

        # score = 객체 인식 스코어
        # class_idx = 객체 클래스 (0: car, 1: pedestrian, 2: cyclist)


        """
        클래스별 타입 문자열 및 기본 높이(h) 설정
        """

        # type_str = 객체 클래스 문자열 (소문자 사용, cyclist는 pedestrian으로 취급)
        # h = car일 경우 1.8, pedestrian일 경우 1.4로 설정


        """
        모델 출력값(픽셀/격자 단위)을 실제 미터(m) 단위로 변환
        픽셀 좌표에서 (x, y)는 라이다 좌표에서 (y, x)에 대응
        """

        x = None
        y = None
        z = -1.55 # KITTI 센서 높이를 고려한 고정 z축 값

        w = None
        l = None


        """
        BEV(Bird's Eye View) 지도상 좌표를 실제 세계 좌표계로 사영
        
        픽셀 좌표에서 세계 좌표로 사영을 할 때, 정규화 된 값을 원래의 값으로 스케일링 해주어야함
        스케일링 수식:
            스케일링 된 값 = 정규화 된 값 / 이미지 크기 (608) * (Max - min) + min
            
            x의 Max 값 : bc[maxX]
            x의 min 값 : bc[minX]
            y의 Max 값 : bc[maxY]
            y의 min 값 : bc[minY]

            주의사항 : w, l의 경우 스케일링 후 - 0.3 해주어야 딱 맞는 박스 시각화 가능
        """
        bc = cnf.boundary

        # x = 스케일링 된 x 값
        # y = 스케일링 된 y 값
        # w = 스케일링 된 w 값
        # l = 스케일링 된 l 값


        """
        Complex-YOLO의 특성인 복소수(im, re)를 이용한 Yaw(회전각) 계산
        yaw = - arctan(im, re)
        """

        im = None
        re = None
        yaw = None

        # pred_list.append(형식에 맞는 리스트)


    return pred_list

def saveDetResult(seq, scene, results):
    """
    검출된 객체 정보를 텍스트 파일로 저장합니다.

    Args:
        seq (int/str): 데이터의 시퀀스 번호 (예: 0, "0000").
        scene (int/str): 장면(프레임) 번호 (예: 0, "000000").
        results (list): inferenceComplexYOLO에서 반환된 객체 정보 리스트.
    """
    save_path = Path(f"data/sample/detection/complex_yolo/{seq}/{scene}.txt")
    save_path.parent.mkdir(parents=True, exist_ok=True)

    f = open(save_path, "w")
    for result in results:
        # x, y, z, w, h, l, yaw, score, type_str 순서로 저장
        x, y, z, w, h, l, yaw, score, type_str = result
        f.write(f"{x} {y} {z} {w} {h} {l} {yaw} {score} {type_str}\n")
    f.close()
    print(f"Detection result saved to {save_path}")


def main():
    # 데이터 경로 및 파일명 설정
    seq_num = 0
    scene_num = 0
    img_file = os.path.join("./data/sample/image_02", f"{seq_num:04d}", f"{scene_num:06d}.png")
    lidar_file = os.path.join("./data/sample/velodyne", f"{seq_num:04d}", f"{scene_num:06d}.bin")

    # 데이터 로드
    lidar_points = np.fromfile(lidar_file, dtype=np.float32).reshape(-1, 4)

    # 모델 선언 및 추론
    model = ComplexYOLO()
    predictions = inferenceComplexYOLO(model, lidar_points)

    # 시각화 준비
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Complex-YOLO OBB Result", width=1280, height=720)

    # 포인트 클라우드 시각화

    """
    TODO
    ComplexYOLO 인식 결과를 활용하여,
    car, pedestrian의 bounding box 시각화

    두 객체의 바운딩 박스 색상은 다르게 지정
    """

    for obj in predictions:
        x, y, z, w, h, l, yaw, score, type_str = obj

        # 각 검출 객체 별 시각화


if __name__ == '__main__':
    main()