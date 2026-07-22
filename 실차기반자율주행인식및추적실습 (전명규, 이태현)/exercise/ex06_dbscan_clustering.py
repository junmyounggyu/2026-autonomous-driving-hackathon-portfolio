import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from exercise.ex03_open3d_basic import create_pcd, load_kitti_bin
from exercise.ex05_ransac_plane import RANSAC

import numpy as np
import open3d as o3d

def get_color_by_id(id:int):
    def hsv_to_rgb(h, s, v):
        if s == 0.:
            return (v, v, v)
        
        i = int(h * 6.0) # 구간 인덱스 (0~5)
        f = (h * 6.0) - i
        p = v * (1.0 - s)
        q = v * (1.0 - s * f)
        t = v * (1.0 - s * (1.0 - f))

        i = i % 6

        if i == 0: return (v, t, p)
        if i == 1: return (q, v, p)
        if i == 2: return (p, v, t)
        if i == 3: return (p, q, v)
        if i == 4: return (t, p, v)
        if i == 5: return (v, p, q)
        
        return (0., 0., 0.)
    
    return hsv_to_rgb((0.618033988749895 * id) % 1., 0.8, 0.95)


def DBSCAN(points, eps, min_pts):
    """
    Args:
        points: 전체 점들 (n,3)
        eps: 주변 점으로 선택할 반경
        min_pts: 군집으로 선택할 최소 점의 수
    
    Returns:
        labels: 각 점의 라벨 인덱스 (n,)
    """
    def find_neighbor_indices(points, idx, eps):
        """
        Args:
            points: 전체 점들 (n,3)
            idx: points 중 몇 번째 인덱스의 주변 점들을 구할지
            eps: 주변 점으로 선택할 반경
        
        Returns:
            indices: 주변 점들의 인덱스
        """
        
        """
        쿼리된 점 기준으로 반경 eps 내에 있는 점들의 인덱스 구하기
        """
        # --- [작성된 코드] ---
        query_point = points[idx]
        
        # NumPy의 브로드캐스팅을 활용해 모든 점과 현재 점 사이의 유클리디안 거리 한 번에 계산
        distances = np.linalg.norm(points - query_point, axis=1)
        
        # 거리가 eps(반경) 이하인 점들의 인덱스만 추출
        indices = np.where(distances <= eps)[0]
        # ----------------------

        return indices
    
    n_points = points.shape[0]
    labels = np.full(n_points, -2) # 미확인 된 점은 -2, 시드가 될 수 없는 점은 -1, 0~ 는 라벨
    cluster_id = 0

    for i in range(n_points):
        """
        이미 확인된 점인지 체크
        """
        # --- [작성된 코드] ---
        # 이미 방문해서 군집에 속했거나(-1 이상의 값) 노이즈로 판별되었다면 건너뜁니다.
        if labels[i] != -2:
            continue
        # ----------------------
        
        neighbors = find_neighbor_indices(points, i, eps)

        if len(neighbors) < min_pts:
            labels[i] = -1
        else:
            labels[i] = cluster_id

            """
            이웃한 점들을 큐에 넣고 현재 id를 부여해 나가기
            이웃한 점들에 대해서도 주변 점을 확인 -> 만약 해당 점의 주변 점 수가 적다면 확장 멈추기
            이웃한 점들에 대해서도 주변 점을 확인 -> 만약 해당 점의 주변 점 수가 충분하다면 해당 점의 이웃 점들도 큐에 집어넣기
            """
            # --- [작성된 코드] ---
            # 확장을 위해 이웃 점들을 큐(리스트)에 담고 순차적으로 탐색합니다.
            queue = list(neighbors)
            q_idx = 0
            
            while q_idx < len(queue):
                curr_idx = queue[q_idx]
                q_idx += 1
                
                # 1. 예전에 노이즈(-1)로 판별되었던 점이라면, 경계점(Border)으로 현재 군집에 포함
                if labels[curr_idx] == -1:
                    labels[curr_idx] = cluster_id
                    
                # 2. 한 번도 확인하지 않은(-2) 점이라면
                elif labels[curr_idx] == -2:
                    # 현재 군집에 포함
                    labels[curr_idx] = cluster_id
                    
                    # 새로운 점의 주변 이웃 탐색
                    new_neighbors = find_neighbor_indices(points, curr_idx, eps)
                    
                    # 주변 이웃이 기준치 이상이면 이 점도 핵심점(Core)이므로 큐에 추가하여 영역 계속 확장
                    if len(new_neighbors) >= min_pts:
                        queue.extend(new_neighbors)
            # ----------------------
            
            cluster_id += 1 # 한 라벨에 대해 확장이 모두 끝났으니 id 1 증가시키고 다음 점에 대해서 확인

    return labels


def main():
    pcd = load_kitti_bin("./data/sample/velodyne/0000/000000.bin")
    pcd = create_pcd(pcd)
    downpcd = pcd.voxel_down_sample(voxel_size=0.2)
    
    downpcd_np = np.asarray(downpcd.points)
    ground_pcd, other_pcd = RANSAC(downpcd_np, 100, 0.2)

    other_pcd_np = np.asarray(other_pcd.points)
    
    # RANSAC으로 바닥이 지워진 나머지 점들(장애물들)을 대상으로 클러스터링 수행
    # eps=0.5 (반경 50cm), min_pts=5 (최소 점 5개 이상 모여야 하나의 덩어리로 인정)
    labels = DBSCAN(other_pcd_np, 0.5, 5)

    colors = np.zeros_like(other_pcd_np)
    for i in range(len(labels)):
        # 노이즈(-1)가 아닌 정상 군집에 속한 점들만 색상 부여
        if labels[i] >= 0:
            colors[i, :] = get_color_by_id(labels[i])
    
    clustered_pcd = create_pcd(other_pcd, colors)

    o3d.visualization.draw_geometries([clustered_pcd])


if __name__=="__main__":
    main()