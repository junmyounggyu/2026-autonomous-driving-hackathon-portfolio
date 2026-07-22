import os
import sys
from pathlib import Path

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import torch
import numpy as np

from model.pointpillars.model import PointPillars
from model.pointpillars.utils import read_points, vis_pc, filter_and_keep_bbox


def loadPointPillars():
    CLASSES = {
        'Pedestrian': 0,
        'Cyclist': 1,
        'Car': 2
    }

    model = PointPillars(nclasses=len(CLASSES))
    model.load_state_dict(torch.load('./model/pth/PointPillars.pth', map_location=torch.device('cpu')))

    model.eval()

    return model

def saveDetResult(lidar_path:str, results:dict):
    """
    검출된 객체 정보를 텍스트 파일로 저장합니다.

    Args:
        lidar_path (str): 라이다 포인트 클라우드 파일 경로
        results (dict): inferencePointPillars 반환된 객체 정보 딕셔너리.
    """
    path_without_ext = os.path.splitext(lidar_path)[0]
    seq_scene = "/".join(path_without_ext.split('/')[-2:])
    save_path = Path(f"data/sample/detection/pointpillars/{seq_scene}.txt")
    save_path.parent.mkdir(parents=True, exist_ok=True)

    f = open(save_path, "w")

    for idx in range(len(results['lidar_bboxes'])):
        # x, y, z, w, h, l, yaw, score, type_str 순서로 저장
        bbox = results['lidar_bboxes'][idx]

        x = bbox[0]
        y = bbox[1]
        z = bbox[2]
        w = bbox[3]
        l = bbox[4]
        h = bbox[5]
        yaw = bbox[6]

        score = results['scores'][idx]

        label = results['labels'][idx]
        type_str = -1
        if label == 0:
            type_str = "pedestrian"
        elif label == 1:
            type_str = "pedestrian"
        elif label == 2:
            type_str = "car"

        f.write(f"{x} {y} {z} {w} {h} {l} {yaw} {score} {type_str}\n")
    f.close()
    print(f"Detection result saved to {save_path}")


def inferencePointPillars(model, lidar_path):
    pts = read_points(lidar_path)
    pts = pts[(pts[:, 0] > 0) & (pts[:, 1] > -39.68) & (pts[:, 2] > -3) & (pts[:, 0] < 69.12) & (pts[:, 1] < 39.68) & (pts[:, 2] < 1)]
    pc_torch = torch.from_numpy(pts)

    with torch.no_grad():
        result_filter = model(batched_pts=[pc_torch],mode='test')[0]

    result_filter = filter_and_keep_bbox(result_filter, np.array([0, -39.68, -3, 69.12, 39.68, 1]))

    # 인식 결과를 저장합니다.
    saveDetResult(lidar_path, result_filter)

    """
    If you want to see the result, uncomment codes below
    """
    # lidar_bboxes = result_filter['lidar_bboxes']
    # labels, scores = result_filter['labels'], result_filter['scores']

    # vis_pc(pts, bboxes=lidar_bboxes, labels=labels)


if __name__ == '__main__':
    seq_num = 0
    scene_num = 0

    lidar_path = f"data/sample/velodyne/{seq_num:04d}/{scene_num:06d}.bin"

    model = loadPointPillars()
    inferencePointPillars(model, lidar_path)
