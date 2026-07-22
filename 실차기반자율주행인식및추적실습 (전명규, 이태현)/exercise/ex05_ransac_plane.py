import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from exercise.ex03_open3d_basic import create_pcd, load_kitti_bin

import numpy as np
import open3d as o3d

def RANSAC(points:np.ndarray, n_iter:int, dist_threshold:float):
    """
    Args:
        points : np.ndarray (N, 3)으로 모든 점 데이터
        n_iter : 샘플링할 횟수
        dist_threshold : inlier로 분류할 점들의 평면까지의 거리 값
    
    Returns:
        (ground_pcd, other_pcd) : 지면에 해당하는 점, 이외의 점
    """
    def plane_fitting(p1, p2, p3):
        """
        Args:
            p1 : np.ndarray (3,)
            p2 : np.ndarray (3,)
            p3 : np.ndarray (3,)
        
        Returns:
            (a, b, c, d) : ax + by + cz + d = 0의 계수들
        """
        
        """
        샘플링한 3개의 점을 기반으로 평면 피팅
        """
        # --- [작성된 코드] ---
        # 1. 두 점을 연결하는 벡터 2개를 만듭니다.
        v1 = p2 - p1
        v2 = p3 - p1
        
        # 2. 두 벡터의 외적(Cross Product)을 구하면 평면의 수직 법선 벡터(Normal Vector)가 나옵니다.
        # 이 수직 벡터의 x, y, z 성분이 바로 평면 방정식의 a, b, c가 됩니다.
        normal = np.cross(v1, v2)
        a, b, c = normal
        
        # 3. 평면의 방정식(ax + by + cz + d = 0)에 p1(x, y, z)을 대입하여 d를 구합니다.
        # d = -(ax + by + cz) 이므로, 법선 벡터와 p1의 내적(Dot Product)에 마이너스를 붙인 것과 같습니다.
        d = -np.dot(normal, p1)
        # ----------------------

        return a, b, c, d
    
    def get_inlier_indices(points, dist_threshold, a, b, c, d):
        """
        Args:
            points : np.ndarray (N, 3)으로 모든 점 데이터
            dist_threshold : inlier로 분류할 점들의 평면까지의 거리 값
            a, b, c, d : ax + by + cz + d = 0 의 계수
        
        Returns:
            (inliers, outliers) : points의 점 중 inlier와 outlier에 해당하는 인덱스들
        """

        """
        각 점들의 평면까지의 거리를 구하고
        설정값 이내로 들어오는 점들의 인덱스를 inliers로 반환
        이외의 점들은 outliers로 반환
        """
        # --- [작성된 코드] ---
        # 점 (x, y, z)와 평면 ax + by + cz + d = 0 사이의 수직 거리 공식:
        # Distance = |ax + by + cz + d| / sqrt(a^2 + b^2 + c^2)
        
        # 1. 분모 계산: 법선 벡터(a,b,c)의 길이(Norm)
        norm_val = np.linalg.norm([a, b, c])
        
        # 예외 처리: 만약 뽑힌 세 점이 일직선상에 있어서 평면이 안 만들어진 경우 (norm이 0)
        if norm_val < 1e-6:
            return np.array([]), np.arange(len(points))
            
        # 2. 분자 및 최종 거리 계산 (NumPy를 이용해 수만 개의 점을 한 번에 계산!)
        distances = np.abs(a * points[:, 0] + b * points[:, 1] + c * points[:, 2] + d) / norm_val
        
        # 3. 임계값(dist_threshold) 이내인 점은 Inlier(바닥), 밖인 점은 Outlier(장애물)로 분류
        inliers = np.where(distances <= dist_threshold)[0]
        outliers = np.where(distances > dist_threshold)[0]
        # ----------------------

        return inliers, outliers
    
    # 최대값을 확인하고 inlier가 최대일 경우의 값들을 저장
    max_inliers = 0
    final_inliers = None
    final_outliers = None

    for _ in range(n_iter):
        """
        3개의 점들을 무작위로 추출
        
        plane_fitting 함수를 활용해 평면 방정식 구하기

        구해진 평면의 방정식에 기반해 inlier와 outlier의 수를 구해 최대인지 확인
        """
        # --- [작성된 코드] ---
        # 1. 수만 개의 점들 중에서 중복 없이 무작위로 인덱스 3개 뽑기
        idx = np.random.choice(len(points), 3, replace=False)
        p1, p2, p3 = points[idx]
        
        # 2. 뽑은 3개의 점으로 가설 평면 만들기
        a, b, c, d = plane_fitting(p1, p2, p3)
        
        # 3. 이 평면에 동의하는(가까이 있는) 점들 걸러내기
        inliers, outliers = get_inlier_indices(points, dist_threshold, a, b, c, d)
        
        # 4. 방금 세운 가설이 지금까지의 최고 기록(가장 많은 Inlier)을 깼다면 갱신!
        if len(inliers) > max_inliers:
            max_inliers = len(inliers)
            final_inliers = inliers
            final_outliers = outliers
        # ----------------------
    
    ground_pcd = create_pcd(points[final_inliers], (0, 1, 0))  # 바닥은 초록색
    other_pcd = create_pcd(points[final_outliers], (1, 0, 0))  # 장애물은 빨간색

    return ground_pcd, other_pcd


def main():
    pcd = load_kitti_bin("./data/sample/velodyne/0000/000000.bin")
    pcd = create_pcd(pcd)
    
    # 3D 점이 너무 많으면 RANSAC 계산이 오래 걸리므로, 0.2m(20cm) 크기로 묶어서 다운샘플링 진행
    downpcd = pcd.voxel_down_sample(voxel_size=0.2)
    
    downpcd_np = np.asarray(downpcd.points)
    
    # RANSAC 실행: 총 100번 가설을 세우고, 바닥 평면에서 0.2m(20cm) 이내에 있는 점들을 바닥으로 인정함
    ground_pcd, other_pcd = RANSAC(downpcd_np, 100, 0.2)

    o3d.visualization.draw_geometries([ground_pcd, other_pcd])


if __name__=="__main__":
    main()