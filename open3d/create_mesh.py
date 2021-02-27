import time
import os
import json
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import open3d as o3d

join = os.path.join

# import fusion

def read_rgbd_image(color_file, depth_file, convert_rgb_to_intensity, config):
    color = o3d.io.read_image(color_file)
    depth = o3d.io.read_image(depth_file)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color,
        depth,
        depth_scale=config["depth_scale"],
        depth_trunc=config["max_depth"],
        convert_rgb_to_intensity=convert_rgb_to_intensity)
    return rgbd_image

def integrate_rgb_frames_for_fragment(color_files, depth_files,
                                        pose_graph, intrinsic,
                                        seq,config):
    # pose_graph = o3d.io.read_pose_graph(pose_graph_name)
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=config["tsdf_cubic_size"] / 512.0,
        sdf_trunc=0.04,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)

    for i in range(0,len(seq),int(config["skip_frames"])):
        print("Frame %03d / %03d" % (i,len(seq)-1))
        rgbd = read_rgbd_image(color_files[i], depth_files[i], False, config)
        pose = pose_graph[pose_graph[:,0]==seq[i],:].reshape((-1))
        if len(pose) == 0: 
            continue        
        t = pose[1:4]
        r = R.from_quat(pose[4:])
        Rot = (r.as_matrix())
        camMat = np.vstack([np.hstack([Rot,t.reshape((3,1))]),[[0,0,0,1]]])
        volume.integrate(rgbd, intrinsic, np.linalg.inv(camMat))
    print("Compute a triangle mesh...")
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    return mesh

def make_pointcloud_for_fragment(path_dataset, color_files, depth_files,
                                 pose_graph, intrinsic, seq,config):

    mesh = integrate_rgb_frames_for_fragment(color_files, depth_files,
                                            pose_graph, intrinsic,
                                            seq,config)

    # pcd = o3d.geometry.PointCloud()
    # pcd.points = mesh.vertices
    # pcd.colors = mesh.vertex_colors
    # print("Saving point cloud to ptcloud.ply...")
    # pcd_name = join(path_dataset,"ptcloud")
    # o3d.io.write_point_cloud(pcd_name, pcd, False, True)    
    # o3d.visualization.draw_geometries([mesh],
    #                                 front=[0.5297, -0.1873, -0.8272],
    #                                 lookat=[2.0712, 2.0312, 1.7251],
    #                                 up=[-0.0558, -0.9809, 0.1864],
    #                                 zoom=0.47)    
    mesh_name = join(path_dataset, config["mesh"])
    o3d.io.write_triangle_mesh(mesh_name, mesh, False, True)                                    

if __name__ == "__main__":

    path_dataset = r'data\frames'
    config_path = r'open3d\config.json'

    associated = np.loadtxt(join(path_dataset,'associated.txt'),dtype=str,delimiter=' ')
    seq = np.array(associated[:,0],dtype=float)

    with open(join(path_dataset,'intrinsic.json'),'r') as f:
        calib = json.load(f)
    
    with open(config_path,'r') as f:
        config = json.load(f)        
    # w, h, fx, fy, cx, cy = np.loadtxt(os.path.join(path_dataset,'calibration.txt'))
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.intrinsic_matrix = np.array(calib['intrinsic_matrix']).reshape((3,3)).T
    intrinsic.height = calib['height']
    intrinsic.width  = calib['width']
    # seq = seq[::int(config["skip_frames"])]

    pose_graph = np.loadtxt('log\KeyFrameTrajectory.txt')
    color_files = [join(path_dataset,associated[i,1]) for i in range(associated.shape[0])]
    depth_files = [join(path_dataset,associated[i,3]) for i in range(associated.shape[0])]

    make_pointcloud_for_fragment(path_dataset, color_files, depth_files,
                                 pose_graph, intrinsic, seq, config)
