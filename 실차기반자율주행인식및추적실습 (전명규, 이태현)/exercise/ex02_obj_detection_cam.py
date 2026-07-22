import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import cv2
from pathlib import Path
from ultralytics import YOLO

def inferenceYOLO(model, img_path, conf_th=0.5):
    """
    Args:
        model = 인퍼런스 할 모델
        img_path = 인퍼런스 할 이미지 경로
        conf_th = confidence 문턱값
    """
    """
    TODO 1: 필터링을 포함한 모델 인퍼런스 코드 완성하기
    """
    predictions = model.predict(source=img_path, device="cpu", verbose=False)
    prediction = predictions[0]

    TARGET_CLASSES = ["car", "person"]
    results = []

    for box in prediction.boxes:
        # -----------------------------------------------------------
        # TODO 1.1: Confidence 기반 필터링
        # 컨피던스 스코어를 기반으로 conf_th 이하의 인식들을 필터링
        # -----------------------------------------------------------

        conf = box.conf.item()

        # Your Code Here
        if conf < conf_th:
            continue


        # -----------------------------------------------------------
        # TODO 1.2: Class Filtering & Mapping
        # 1. 인식된 박스의 클래스를 찾기
        # 2. 인식된 박스의 클래스와 TARGET_CLASSES를 비교하여 필터링 해내기
        # 3. 만일 클래스의 이름이 person이라면 pedestrian으로 변경하기 *라이다 인식 결과와 동일하게 맞추기 위함
        # -----------------------------------------------------------

        # Your Code Here
        cls_idx = int(box.cls.item())
        cls_name = model.names[cls_idx]

        if cls_name not in TARGET_CLASSES:
            continue
            
        if cls_name == "person":
            cls_name = "pedestrian"



        # -----------------------------------------------------------
        # TODO 1.3: Coordinate Extraction
        # x, y, w, h 를 찾아서 요구하는 포맷으로 가공하여 결과 리스트에 추가하기
        # -----------------------------------------------------------

        # Your Code Here (x, y, w, h = ...)
        x, y, w, h = box.xywh[0].tolist()

        results.append([x, y, w, h, conf, cls_name])



    return results

def saveDetResult(seq, scene, results):
    """
    Args:
        seq : sequence number
        scene : scene number
        results : 인식 결과 리스트 (list of [x, y, w, h, conf, cls_name])
    """
    save_path = Path(f"data/sample/detection/complex/{seq}/{scene}.txt")

    save_path.parent.mkdir(parents=True, exist_ok=True)

    f = open(save_path, "w")

    for result in results:
        x, y, w, h, conf, cls_name = result

        f.write(f"{x} {y} {w} {h} {conf} {cls_name}\n")

    f.close()
    print(f"Detection result saved to {save_path}")

def main():
    # load model
    model = YOLO("model/pth/yolov5x.pt")

    seq_num = 0
    scene_num = 0
    img_path = f"data/sample/image_02/{seq_num:04d}/{scene_num:06d}.png"

    # Step 1: 앞서 구현한 인퍼런스 코드를 통해 모델 추론
    detections = inferenceYOLO(model, img_path, conf_th=0.5)

    # 객체 인식 결과를 저장하고 싶다면 아래의 코드의 주석을 해제
    # saveDetResult(seq_num, scene_num, detections)

    # 시각화를 위한 이미지 로드
    img_vis = cv2.imread(img_path)

    for result in detections:
        x, y, w, h, conf, cls_name = result

        # -----------------------------------------------------------
        # TODO 2.1: Coordinate Transformation
        # OpenCV활용을 위하여 센터 기반의 좌표인 (x, y, w, h)를 콘너 기반의 좌표인 (x1, y1, x2, y2)로 변경
        # -----------------------------------------------------------

        x1 = int(x - w / 2)
        y1 = int(y - h / 2)
        x2 = int(x + w / 2)
        y2 = int(y + h / 2)


        # -----------------------------------------------------------
        # TODO 2.2: Drawing
        # 1. 초록색 사각형을 그리세요 *초록: (0, 255, 0).
        # 2. 클래스 이름과 confidence를 텍스트 라벨로 추가
        # -----------------------------------------------------------

        # cv2.rectangle(img_vis, (x1, y1), (x2, y2), 색상, 굵기)
        cv2.rectangle(img_vis, (x1, y1), (x2, y2), (0, 255, 0), 1)
        
        # cv2.putText(img_vis, 클래스이름, 글자 시작 위치, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 색상, 굵기)
        label = f"{cls_name} {conf:.2f}"
        text_y = max(y1 - 10, 10)
        cv2.putText(img_vis, label, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)


    # Step 2: 최종 결과물 시각화
    cv2.imshow("YOLO Filtered Results", img_vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
