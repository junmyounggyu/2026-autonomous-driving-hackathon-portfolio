import os
import sys

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
import open3d as o3d

from exercise.ex09_kalman_filter import KalmanFilter
from exercise.ex03_open3d_basic import load_kitti_bin, create_pcd
from exercise.ex07_obj_detection_lidar import inferenceComplexYOLO

from tqdm import tqdm
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from model.utils.complexYOLO_utils import ComplexYOLO

seqmap = {}
seqmap[0] = 154
seqmap[1] = 447
seqmap[2] = 233
seqmap[3] = 144
seqmap[4] = 314
seqmap[5] = 297
seqmap[6] = 270
seqmap[7] = 800
seqmap[8] = 390
seqmap[9] = 803
seqmap[10] = 294
seqmap[11] = 373
seqmap[12] = 78
seqmap[13] = 340
seqmap[14] = 106
seqmap[15] = 376
seqmap[16] = 209
seqmap[17] = 145
seqmap[18] = 339
seqmap[19] = 1059
seqmap[20] = 837

def create_oriented_box(x, y, z, box_size, hdg, color):
    center = np.array([x, y, z + box_size[1] / 2])
    extent = np.array([box_size[2], box_size[0], box_size[1]])
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle([0, 0, hdg])
    obb = o3d.geometry.OrientedBoundingBox(center, rotation_matrix, extent)
    obb.color = color
    line_set = o3d.geometry.LineSet.create_from_oriented_bounding_box(obb)

    return line_set


def read_calib(calib_path):
    P2 = None
    vtc_mat = None
    R0 = None

    with open(calib_path) as f:
        for line in f:
            line = line.strip()
            if not line: continue
            
            parts = line.split()
            key = parts[0].rstrip(':')
            data = np.array(parts[1:], dtype=np.float32)

            if key == "P2":
                P2 = data.reshape(3, 4)
            
            elif key in ["Tr_velo_to_cam", "Tr_velo_cam"]:
                vtc_mat = data.reshape(3, 4)
                vtc_mat = np.vstack([vtc_mat, [0, 0, 0, 1]])
            
            elif key in ["R0_rect", "R_rect"]:
                R0_3x3 = data.reshape(3, 3)
                R0 = np.eye(4, dtype=np.float32)
                R0[:3, :3] = R0_3x3

    if R0 is not None and vtc_mat is not None:
        vtc_mat = np.matmul(R0, vtc_mat)
    
    return P2, vtc_mat


def read_detection(file):
    def is_float(value):
        try:
            float(value)
            return True
        except ValueError:
            return False
    
    data = []
    with open(file, 'r') as f:
        for line in f:
            line = line.strip()
            if not line: continue
            
            row = line.split()
            converted_row = [float(x) if is_float(x) else x for x in row]
            data.append(converted_row)

    return data


def lidar_to_kitti_format(lidar_box, P2, vtc_mat, img_shape=(375, 1242)):
    x, y, z, hdg, w, h, l = lidar_box

    lidar_bottom_center = np.array([x, y, z, 1.0])
    cam_bottom_center = np.matmul(vtc_mat, lidar_bottom_center)
    cam_x, cam_y, cam_z = cam_bottom_center[:3]

    if cam_z <= 0:
        return None

    x_corners = [l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2]
    y_corners = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2]
    z_corners = [h, h, h, h, 0, 0, 0, 0]
    
    c, s = np.cos(hdg), np.sin(hdg)
    corners_lidar = np.zeros((4, 8))
    for i in range(8):
        corners_lidar[0, i] = c * x_corners[i] - s * y_corners[i] + x
        corners_lidar[1, i] = s * x_corners[i] + c * y_corners[i] + y
        corners_lidar[2, i] = z_corners[i] + z
        corners_lidar[3, i] = 1.0

    pts_cam = np.matmul(vtc_mat, corners_lidar)
    
    if np.all(pts_cam[2, :] <= 0):
        return None

    pts_2d = np.matmul(P2, pts_cam)
    pts_2d[:2, :] /= pts_2d[2, :]
    
    u_min = np.min(pts_2d[0, :])
    v_min = np.min(pts_2d[1, :])
    u_max = np.max(pts_2d[0, :])
    v_max = np.max(pts_2d[1, :])

    img_h, img_w = img_shape
    if u_max < 0 or v_max < 0 or u_min >= img_w or v_min >= img_h:
        return None

    u_min_clip = np.clip(u_min, 0, img_w - 1)
    v_min_clip = np.clip(v_min, 0, img_h - 1)
    u_max_clip = np.clip(u_max, 0, img_w - 1)
    v_max_clip = np.clip(v_max, 0, img_h - 1)

    if (u_max_clip - u_min_clip) < 2 or (v_max_clip - v_min_clip) < 2:
        return None

    ry = -hdg - np.pi / 2
    ry = (ry + np.pi) % (2 * np.pi) - np.pi

    return {
        'bbox': [float(u_min_clip), float(v_min_clip), 
                 float(u_max_clip), float(v_max_clip)],
        'dimensions': [float(h), float(w), float(l)],
        'location': [float(cam_x), float(cam_y), float(cam_z)],
        'rotation_y': float(ry)
    }


class Obj:
    def __init__(self, init_X, init_P, init_Q, z, box_size, cls):
        assert isinstance(box_size, list) and len(box_size) == 3
        assert isinstance(cls, str) and cls in ["car", "pedestrian"]

        self.kf = KalmanFilter(init_X, init_P, init_Q)
        self.cls = cls
        self.box_size = box_size
        self.z = z

        self.alive_cnt = 0
        self.remove_cnt = 0

    def predict(self):
        self.kf.predict_step()
        self.remove_cnt += 1
    
    def update(self, Z, R, z, box_size):
        self.kf.update_step(Z, R)
        self.box_size = box_size
        self.z = z
        self.remove_cnt = 0
        self.alive_cnt += 1

    @property
    def x(self):
        return self.kf.X[0, 0]
    
    @property
    def y(self):
        return self.kf.X[1, 0]
    
    @property
    def hdg(self):
        return self.kf.X[2, 0]


def track_seq(seq, visualization_flag=False):
    ########################
    TRACKER_NAME = "test"
    detection_model = "point_pillars"
    P_DEFAULT = [1.0, 1.0, 0.2, 100.0, 100.0, 10.0]
    Q_DEFAULT = [0.1, 0.1, 0.05, 1.0, 1.0, 1.0]
    R_DEFAULT = [0.01, 0.01, 5.0]
    N_REMOVE = 7
    N_ACTIVE = 2
    DIST_MATCH = 2.0
    ########################

    if detection_model is None:
        model = ComplexYOLO()
    else:
        assert isinstance(detection_model, str)
        detector_root = "./data/sample/detection/"+detection_model+"/%04d"%seq

    velo_root = "./data/sample/velodyne/%04d"%seq
    calib_path = "./data/sample/calib/%04d.txt"%seq
    result_root = "./data/result/"+TRACKER_NAME+"/data"
    os.makedirs(result_root, exist_ok=True)

    result_text = ""

    P2, vtc_mat = read_calib(calib_path)

    # n_frame = len(os.listdir(velo_root))

    track_car_list = dict()
    track_ped_list = dict()
    obj_id_val = 0
    
    for i in tqdm(range(seqmap[seq])):
        file_path = velo_root + "/%06d.bin"%i

        
        if detection_model is None:
            pcd = load_kitti_bin(file_path, True)
            res = inferenceComplexYOLO(model, pcd)
        else:
            res = read_detection(detector_root + "/%06d.txt"%i)

        detect_car_list = []
        detect_ped_list = []
        for detect in res:
            if detect[8] == "car":
                detect_car_list.append(detect)
            elif detect[8] == "pedestrian":
                detect_ped_list.append(detect)
            else:
                print("unknown class")

        if len(detect_car_list) == 0:
            for oid, track in track_car_list.items():
                track.predict()
        elif len(track_car_list) == 0:
            P = P_DEFAULT
            Q = Q_DEFAULT
            for detect in detect_car_list:
                track_car_list[obj_id_val] = Obj(detect[0:2]+[detect[6]]+[0., 0., 0.], P, Q, detect[2], detect[3:6], detect[8])
                obj_id_val += 1
        else:
            track_ids = []
            trackers_xy = []
            detects_xy = []
            for oid, track in track_car_list.items():
                track.predict()
                track_ids.append(oid)
                trackers_xy.append([track.x, track.y])
            for detect in detect_car_list:
                detects_xy.append([detect[0], detect[1]])

            trackers_xy = np.array(trackers_xy)
            detects_xy = np.array(detects_xy)
            dists = cdist(trackers_xy, detects_xy, metric="euclidean")

            row_ind, col_ind = linear_sum_assignment(dists)

            for j in range(len(row_ind)):
                track = track_car_list[track_ids[row_ind[j]]]
                detect = detect_car_list[col_ind[j]]
                if dists[row_ind[j], col_ind[j]] < DIST_MATCH:
                    R = R_DEFAULT
                    track.update(detect[0:2]+[detect[6]], R, detect[2], detect[3:6])
                else:
                    P = P_DEFAULT
                    Q = Q_DEFAULT
                    track_car_list[obj_id_val] = Obj(detect[0:2]+[detect[6]]+[0., 0., 0.], P, Q, detect[2], detect[3:6], detect[8])
                    obj_id_val += 1
            
            for j, detect in enumerate(detect_car_list):
                if j not in col_ind:
                    P = P_DEFAULT
                    Q = Q_DEFAULT
                    track_car_list[obj_id_val] = Obj(detect[0:2]+[detect[6]]+[0., 0., 0.], P, Q, detect[2], detect[3:6], detect[8])
                    obj_id_val += 1

        remove_track_list = []
        for oid, track in track_car_list.items():
            if track.remove_cnt > N_REMOVE:
                remove_track_list.append(oid)
        for oid in remove_track_list:
            del track_car_list[oid]

        for oid, track in track_car_list.items():
            if track.alive_cnt > N_ACTIVE:
                converted = lidar_to_kitti_format(
                    [track.x, track.y, track.z, track.hdg] + track.box_size,
                    P2, vtc_mat
                )
                if converted is None:
                    continue
                score = 1.0
                track_res = '%d %d %s -1 -1 -10 %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n'%(
                    i,oid,track.cls,converted["bbox"][0],converted["bbox"][1],converted["bbox"][2],
                    converted["bbox"][3],converted["dimensions"][0],converted["dimensions"][1],
                    converted["dimensions"][2],converted["location"][0],converted["location"][1],
                    converted["location"][2],converted["rotation_y"],score)
                result_text += track_res
        
        if visualization_flag:
            plot_list = []
            plot_list.append({'name': 'pcd', 'geometry': create_pcd(pcd[:, :3], (0, 0, 0)), 'point_size': 10.0})
            for oid, track in track_car_list.items():
                plot_list.append({
                    'name': f't{oid}', 
                    'geometry': create_oriented_box(track.x, track.y, track.z, track.box_size, track.hdg, (1, 0, 0)),
                    'line_width': 8.0
                })
            for j, detect in enumerate(detect_car_list):
                plot_list.append({
                    'name': f'd{j}', 
                    'geometry': create_oriented_box(detect[0], detect[1], detect[2], detect[3:6], detect[6], (0, 0, 1)),
                    'line_width': 8.0
                })
                plot_list.append(create_oriented_box(detect[0], detect[1], detect[2], detect[3:6], detect[6], (0, 0, 1)))

            o3d.visualization.draw(plot_list,
                                lookat=[-2, 0.0, -0.5],
                                eye=[-20.0, 0.0, 15.0],
                                up=[0.0, 0.0, 1.0],
                                field_of_view=60.0,
                                show_skybox=False
            )

    with open(result_root + "/%04d.txt"%seq, 'w', encoding='utf-8') as f:
        f.write(result_text)


def main():
    for i in range(21):
        track_seq(i)


if __name__=="__main__":
    main()