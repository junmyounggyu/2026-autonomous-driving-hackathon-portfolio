import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import cv2
import numpy as np
import open3d as o3d

from model.utils.kitti_utils import read_calib_file

def main():
    # ==============================================================================
    # SECTION 1: DATA LOADING & CALIBRATION SETUP
    # ==============================================================================

    seq_num = 0
    scene_num = 0
    img_path = f"./data/sample/image_02/{seq_num:04d}/{scene_num:06d}.png"
    lidar_path = f"data/sample/velodyne/{seq_num:04d}/{scene_num:06d}.bin"
    calib_path = f"data/sample/calib/{seq_num:04d}.txt"

    img = cv2.imread(img_path)
    lidar_points = np.fromfile(lidar_path, dtype=np.float32).reshape(-1, 4)

    """
    P : Projection Matrix (Cam -> Image)
    V2C : Extrinsic Matrix (LiDAR -> Cam)
    """
    P, V2C = read_calib_file(calib_path)

    img_h, img_w = img.shape[:2]
    pts_3d = lidar_points[:, :3]



   # ==============================================================================
    # SECTION 2: COORDINATE TRANSFORMATION LiDAR --> Pixel
    # ==============================================================================

    # """
    # TODO: 3차원 포인트를 동차좌표계로 변환 
    # 결과를 'pts_3d_hom'에 저장
    # """
    ones = np.ones((pts_3d.shape[0], 1), dtype=np.float32)
    pts_3d_hom = np.hstack([pts_3d, ones]).T


    # """
    # TODO: 라이다 포인트들을 카메라 좌표계로 변환 
    # 결과를 'pts_3d_cam'에 저장
    # """
    pts_3d_cam = V2C @ pts_3d_hom


    # """
    # TODO: 삼차원 포인트들을 이미지 좌표계로 투영
    # 결과를 'pts_2d_img'
    # """
    # [수정된 부분] P 행렬(3x4)과 곱하기 위해 카메라 좌표(3,N) 맨 아래에 1을 덧붙여 동차좌표계(4,N)로 만듭니다.
    ones_row = np.ones((1, pts_3d_cam.shape[1]), dtype=np.float32)
    pts_3d_cam_hom = np.vstack([pts_3d_cam, ones_row])
    
    # 이제 차원이 4로 맞아서 곱셈이 완벽하게 작동합니다!
    pts_2d_img = P @ pts_3d_cam_hom


    # """
    # TODO: 이미지 좌표계에서 정규 픽셀 좌표계로 변환 
    # Transpose를 취해서 결과를 (N, 2) 형태로 'pts_2d_pix'에 저장
    # """
    pts_2d_pix = (pts_2d_img[:2, :] / pts_2d_img[2, :]).T



    # ==============================================================================
    # SECTION 3: FOV FILTERING & DATA EXTRACTION
    # ==============================================================================
    """
    Step 5 - 카메라 FOV기반 마스킹
    아래의 조건에 맞는 포인트들만 남기고 필터링
    1. x >= 0
    2. x < Image Width
    3. y >= 0
    4. y < Image Height
    5. Depth (Z in Camera) > 0
    """
    fov_mask = (pts_2d_pix[:, 0] >= 0) & (pts_2d_pix[:, 0] < img_w) & \
               (pts_2d_pix[:, 1] >= 0) & (pts_2d_pix[:, 1] < img_h) & \
               (pts_3d_cam[2, :] > 0)

    valid_pts_2d = pts_2d_pix[fov_mask].astype(np.int32)
    valid_pts_3d = pts_3d[fov_mask]
    valid_depths = pts_3d_cam[2, fov_mask]



    # ==============================================================================
    # SECTION 4: 2D VISUALIZATION (PROJECTION)
    # ==============================================================================

    img_draw = img.copy()

    # 이미지 위에 변환된 포인트들 그리기
    for i in range(len(valid_pts_2d)):
        depth = valid_depths[i]
        color = (int(255 * min(depth/50, 1)), 255, 0)
        cv2.circle(img_draw, (valid_pts_2d[i, 0], valid_pts_2d[i, 1]), 1, color, -1)

    cv2.imshow("Projected LiDAR Points on Image", img_draw)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


    # SECTION 4까지 완료하였다면 아래의 return을 삭제해주세요
    # return (삭제 완료!)



    # ==============================================================================
    # SECTION 5: 3D VISUALIZATION (COLORIZATION TASK)
    # ==============================================================================
    # """
    # TODO: Step 6 - 포인트 들에 대응하는 픽셀 색상 추출 
    # Hint: 'valid_pts_2d'를 이용해서 'img'에서 BGR colors 추출
    # 주의: 이미지에서 인덱싱할 떄, [row, col] -> [y, x].
    #      이미지에 색상은 BGR로 저장되어 있음, 순서에 유의
    # 결과를 'colors_rgb'에 저장 후 시각화
    # """
    
    # 1. 픽셀 좌표 분리 (u는 x축/열, v는 y축/행)
    u = valid_pts_2d[:, 0]
    v = valid_pts_2d[:, 1]
    
    # 2. 이미지에서 BGR 색상 추출 (이미지는 y축부터 먼저 인덱싱해야 함)
    colors_bgr = img[v, u]
    
    # 3. BGR을 RGB로 뒤집고, Open3D에 맞게 0~1 사이 값으로 정규화(/255.0)
    colors_rgb = colors_bgr[:, ::-1] / 255.0 
    
    # 4. Open3D 시각화
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(valid_pts_3d)
    pcd.colors = o3d.utility.Vector3dVector(colors_rgb)
    
    o3d.visualization.draw_geometries([pcd], window_name="Colorized 3D LiDAR")



if __name__ == '__main__':
    main()