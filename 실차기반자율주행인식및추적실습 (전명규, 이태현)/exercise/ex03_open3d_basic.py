import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import open3d as o3d
import numpy as np

def create_pcd(xyz:np.ndarray, colors:np.ndarray=None):
    """
    NumPy 배열 데이터를 Open3D의 PointCloud 객체로 변환

    Args:
        xyz : (N, 3) 형태의 점 좌표 데이터 또는 기존 PointCloud 객체
        colors : (N, 3) 형태의 RGB 배열 또는 단일 색상 튜플 (R, G, B)

    Returns:
        pcd : 생성된 Open3D 포인트 클라우드 객체
    """
    pcd = o3d.geometry.PointCloud()

    if isinstance(xyz, o3d.cpu.pybind.geometry.PointCloud):
        pcd.points = xyz.points
    elif isinstance(xyz, np.ndarray):
        assert len(xyz.shape) == 2
        assert xyz.shape[1] == 3
        pcd.points = o3d.utility.Vector3dVector(xyz)
    n_pts = len(np.asarray(pcd.points))

    if isinstance(colors, tuple):
        assert len(colors) == 3
        colors_arr = np.tile(colors, (n_pts, 1))
        pcd.colors = o3d.utility.Vector3dVector(colors_arr)
    elif isinstance(colors, np.ndarray):
        assert len(colors.shape) == 2
        assert colors.shape[1] == 3
        assert colors.shape[0] == n_pts
        pcd.colors = o3d.utility.Vector3dVector(colors)
    else:
        pass

    return pcd


def load_kitti_bin(file_path, with_intensity=False)->np.ndarray:
    """
    KITTI 데이터셋의 .bin 파일을 읽어 NumPy 배열로 반환합니다.

    Args:
        file_path : .bin 파일 경로
        with_intensity : True일 경우 반사도(Intensity)를 포함한 (N, 4) 반환, False일 경우 (N, 3) 반환

    Returns:
        raw_data: 로드된 포인트 데이터
    """
    raw_data = np.fromfile(file_path, dtype=np.float32).reshape(-1, 4)
    if with_intensity:
        return raw_data
    else:
        return raw_data[:, :3]


def main_visualization():
    """
    랜덤 포인트 그리기
    """
    # randomly generate N points pointcloud
    N_POINTS = 1000
    random_xyz = (np.random.rand(N_POINTS, 3) - 0.5) * 20
    random_color = np.random.rand(N_POINTS, 3)

    # make PointCloud object
    pcd = o3d.geometry.PointCloud()
    print(type(pcd))
    pcd.points = o3d.utility.Vector3dVector(random_xyz)
    pcd.colors = o3d.utility.Vector3dVector(random_color)

    # visualization
    o3d.visualization.draw_geometries([pcd])


def main_kitti_visualization():
    """
    데이터 읽어서 그려보기
    """
    # KITTI bin 파일 경로 (실제 파일이 있는 경로로 맞춰주세요)
    bin_path = "./data/sample/velodyne/0000/000000.bin"
    
    # 1. 만들어둔 함수로 bin 파일 읽어오기
    kitti_data = load_kitti_bin(bin_path, with_intensity=False)
    
    # 2. NumPy 배열을 Open3D 포인트 클라우드 객체로 변환 (기본 색상: 흰색)
    pcd = create_pcd(kitti_data, (0, 0, 0))
    
    # 3. 화면에 출력
    o3d.visualization.draw_geometries([pcd], window_name="KITTI Point Cloud")


def main_downsampling():
    """
    포인트 다운샘플링 해보기
    """
    pcd = load_kitti_bin("./data/sample/velodyne/0000/000000.bin")
    ori_pcd = create_pcd(pcd, (0, 0, 1))

    # downsample with voxel size [m]
    # 복셀 크기(voxel_size)를 0.2m(20cm)로 설정하여 다운샘플링 진행
    voxel_size = 0.2 
    down_pcd = ori_pcd.uniform_down_sample(every_k_points=10)

    down_pcd = create_pcd(down_pcd, (1, 0, 0))

    print(f"original points num    : {len(np.asarray(ori_pcd.points))}")
    print(f"downsampled points num : {len(np.asarray(down_pcd.points))}")

    vis_a = o3d.visualization.Visualizer()
    vis_b = o3d.visualization.Visualizer()
    vis_a.create_window(window_name='Point Cloud A', width=600, height=600, left=50)
    vis_b.create_window(window_name='Point Cloud B', width=600, height=600, left=700)
    vis_a.add_geometry(ori_pcd)
    vis_b.add_geometry(down_pcd)

    while True:
        view_ctl_a = vis_a.get_view_control()
        cam_params = view_ctl_a.convert_to_pinhole_camera_parameters()

        view_ctl_b = vis_b.get_view_control()
        view_ctl_b.convert_from_pinhole_camera_parameters(cam_params)

        if not vis_a.poll_events(): break
        vis_a.update_renderer()
        
        if not vis_b.poll_events(): break
        vis_b.update_renderer()

    vis_a.destroy_window()
    vis_b.destroy_window()


def boundingBox3DwLines():
    """
    선을 이용해서 3D bounding box 그리기
    """
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="3D Bounding Box", width=1280, height=720)

    # 1. 랜덤 포인트 클라우드 생성
    N_POINTS = 10
    random_xyz = (np.random.rand(N_POINTS, 3) - 0.5) * 20
    random_color = np.random.rand(N_POINTS, 3)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(random_xyz)
    pcd.colors = o3d.utility.Vector3dVector(random_color)

    # 2. PCA 기반 8개 코너점 계산 (아래에 구현된 get_corners 함수 활용)
    corners_3d = get_corners(pcd)

    # 3. 선 연결 정의 (8개 점을 잇는 12개의 모서리)
    # 0~3은 앞/윗면, 4~7은 뒤/아랫면을 의미하며, 점들의 순서에 맞게 인덱스를 연결합니다.
    lines = [
        [0, 1], [1, 2], [2, 3], [3, 0],  # 한 쪽 면 (예: 윗면)
        [4, 5], [5, 6], [6, 7], [7, 4],  # 반대 쪽 면 (예: 아랫면)
        [0, 4], [1, 5], [2, 6], [3, 7]   # 두 면을 연결하는 기둥
    ]

    # 4. LineSet 생성 (박스 그리기)
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(corners_3d)
    line_set.lines = o3d.utility.Vector2iVector(lines)

    # 박스 색상 설정 (예: 초록색)
    line_set.paint_uniform_color([0, 1, 0])

    # 5. 시각화 추가
    vis.add_geometry(pcd)
    vis.add_geometry(line_set)

    vis.run()
    vis.destroy_window()


def get_corners(points):
    """
    주성분 분석(PCA)을 통해 포인트 클라우드를 감싸는 OBB의 8개 코너점 좌표를 계산합니다

    Args:
        points : 입력 포인트 클라우드 객체

    Returns:
        eight_corners : (8, 3) 형태의 월드 좌표계 기준 코너점 배열
    """
    points = np.asarray(points.points)
    mean = np.mean(points, axis=0)
    centered_points = points - mean

    cov = np.cov(centered_points, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)

    projected_points = centered_points @ eigenvectors

    p_min = np.min(projected_points, axis=0)
    p_max = np.max(projected_points, axis=0)

    half_extent = (p_max - p_min) / 2
    local_center = (p_min + p_max) / 2

    x, y, z = half_extent
    offsets = np.array([
        [ x,  y,  z], [ x,  y, -z], [ x, -y, -z], [ x, -y,  z],
        [-x,  y,  z], [-x,  y, -z], [-x, -y, -z], [-x, -y,  z]
    ])

    eight_corners = (local_center + offsets) @ eigenvectors.T + mean

    return eight_corners


def boundingBox3D():
    """
    OrientedBoundingBox 를 이용해서 3D bounding box그리기
    """
    # 1. 시각화 설정
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Manual Oriented Bounding Box", width=1280, height=720)

    # 2. 랜덤 포인트 클라우드 생성
    N_POINTS = 10
    random_xyz = (np.random.rand(N_POINTS, 3) - 0.5) * 20
    random_color = np.random.rand(N_POINTS, 3)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(random_xyz)
    pcd.colors = o3d.utility.Vector3dVector(random_color)

    # 3. 직접 OBB 파라미터 계산 (PCA 방식)
    points = np.asarray(pcd.points)
    mean = np.mean(points, axis=0)
    centered_points = points - mean

    cov = np.cov(centered_points, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)

    R = eigenvectors

    projected_points = centered_points @ R
    p_min = np.min(projected_points, axis=0)
    p_max = np.max(projected_points, axis=0)

    extent = p_max - p_min

    local_center = (p_min + p_max) / 2
    center = local_center @ R.T + mean

    # 4. o3d.geometry.OrientedBoundingBox 함수 활용
    # center, R, extent를 생성자에 입력
    obb = o3d.geometry.OrientedBoundingBox(center, R, extent)
    obb.color = (0, 1, 0)  # 초록색 선

    # 5. 시각화
    vis.add_geometry(pcd)
    vis.add_geometry(obb)

    vis.run()
    vis.destroy_window()

if __name__=="__main__":
    # 실행하고자 하는 함수의 주석(#)을 지우고 테스트해 보세요.
    # main_visualization()
    # main_kitti_visualization()
    # main_downsampling()
    # boundingBox3DwLines()
     boundingBox3D()