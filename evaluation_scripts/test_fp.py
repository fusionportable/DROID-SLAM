import sys
sys.path.append('droid_slam')

import time

from tqdm import tqdm
import numpy as np
import torch
import lietorch
import cv2
import os
import glob 
import time
import argparse
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface
import open3d as o3d

import torch.nn.functional as F
from droid import Droid


def show_image(image):
#     print(image.shape)
    image = image.permute(1, 2, 0).cpu().numpy()
    cv2.imshow('image', image / 255.0)
    cv2.waitKey(1)

def image_stream(image_list, calib, image_size=[240, 320]):
    """ image generator """

    calib = np.loadtxt(calib, delimiter=" ")
    fx, fy, cx, cy = calib[:4]

    K = np.eye(3)
    K[0,0] = fx
    K[0,2] = cx
    K[1,1] = fy
    K[1,2] = cy

    # read image list
    # rgb_list = os.path.join(datapath, rgb_list)
    # rgb_list = np.loadtxt(rgb_list, delimiter=' ', dtype=np.unicode_)
    
    # self.timestamps = rgb_list[:,0].astype(np.float)
    # self.images = [osp.join(datapath, x) for x in rgb_list[:,1]]
    # self.images = self.images[::rate]
    # self.timestamps = self.timestamps[::rate]
    # timestamps = rgb_list[:,0].astype(float)
    # images = [os.path.join(datapath, x) for x in rgb_list[:,1]]

    # image_dir = os.path.join(datapath, 'left')
    # images_list = sorted(glob.glob(os.path.join(image_dir, '*.png')))
      
    for t, imfile in enumerate(image_list):
        image = cv2.imread(imfile)
        if len(calib) > 4:
            image = cv2.undistort(image, K, calib[4:])

        h0, w0, _ = image.shape
        # h1 = int(h0 * np.sqrt((384 * 512) / (h0 * w0)))
        # w1 = int(w0 * np.sqrt((384 * 512) / (h0 * w0)))

        # image = cv2.resize(image, (w1, h1))
        h1, w1 = image_size
        image = cv2.resize(image, (w1, h1))

        # image = image[:h1-h1%8, :w1-w1%8]
        image = torch.as_tensor(image).permute(2, 0, 1)

        intrinsics = torch.as_tensor([fx, fy, cx, cy])
        intrinsics[0::2] *= (image.shape[2] / w0)
        intrinsics[1::2] *= (image.shape[1] / h0)

        yield t, image[None], intrinsics


def parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root")
    parser.add_argument("--weights", default="droid.pth")
    parser.add_argument("--buffer", type=int, default=1024)
    parser.add_argument("--image_size", default=[240, 320])
    parser.add_argument("--disable_vis", action="store_true")
    parser.add_argument("--calib")
    parser.add_argument("--rgb_list")
    
    parser.add_argument("--beta", type=float, default=0.6)
    parser.add_argument("--filter_thresh", type=float, default=1.75)
    parser.add_argument("--warmup", type=int, default=12)
    parser.add_argument("--keyframe_thresh", type=float, default=2.25)
    parser.add_argument("--frontend_thresh", type=float, default=12.0)
    parser.add_argument("--frontend_window", type=int, default=25)
    parser.add_argument("--frontend_radius", type=int, default=2)
    parser.add_argument("--frontend_nms", type=int, default=1)

    parser.add_argument("--backend_thresh", type=float, default=15.0)
    parser.add_argument("--backend_radius", type=int, default=2)
    parser.add_argument("--backend_nms", type=int, default=3)
    # parser.add_argument("--timestamp", type=str, default="tstamps.txt")
    parser.add_argument("--reconstruction_path", help="path to saved reconstruction")
    parser.add_argument("--upsample", action="store_true")

    args = parser.parse_args()

    return args


if __name__ == '__main__':
    args = parser()
    args.stereo = False

    torch.multiprocessing.set_start_method('spawn')

    print("Running evaluation on {}".format(args.root))
    print(args)

    # load image stream
    rgb_list = os.path.join(args.root, args.rgb_list)
    rgb_list = np.loadtxt(rgb_list, delimiter=' ', dtype=np.unicode_)
    image_dir = os.path.join(args.root, 'frame_left')
    images = [os.path.join(image_dir, x) for x in rgb_list[:,1]]
    timestamps = rgb_list[:,0].astype(np.float64)

    if args.reconstruction_path is not None:
        args.upsample = True

    droid = Droid(args)
    time.sleep(5)
    start_time = time.time()
    for (t, image, intrinsics) in tqdm(image_stream(images, args.calib), total=len(images)):
      #   print(image.shape)
        if not args.disable_vis:
            show_image(image[0])
        droid.track(t, image, intrinsics=intrinsics)

    traj_est = droid.terminate(image_stream(images, args.calib))
    end_time = time.time()
    total_runtime = end_time - start_time
    print("Total runtime: ", total_runtime)
    print("FPS: ", len(images) / total_runtime)

    ### run evaluation ###
    print("#"*20 + " Results...")

    traj_est = PoseTrajectory3D(
        positions_xyz=traj_est[:,:3],
        orientations_quat_wxyz=traj_est[:,3:],
        timestamps=np.array(timestamps))

    file_interface.write_tum_trajectory_file(os.path.join(args.root, f"traj_est_droid.txt"), traj_est)

    print("Saved trajectory to {}".format(os.path.join(args.root, f"traj_est_droid.txt")))
    cv2.destroyAllWindows()
    # o3d.visualization.Visualizer.close()


#     print("#"*20 + " Results...")

#     import evo
#     from evo.core.trajectory import PoseTrajectory3D
#     from evo.tools import file_interface
#     from evo.core import sync
#     import evo.main_ape as main_ape
#     from evo.core.metrics import PoseRelation

#     image_path = os.path.join(args.datapath, 'rgb')
#     images_list = sorted(glob.glob(os.path.join(image_path, '*.png')))[::2]
#     tstamps = [float(x.split('/')[-1][:-4]) for x in images_list]

#     traj_est = PoseTrajectory3D(
#         positions_xyz=traj_est[:,:3],
#         orientations_quat_wxyz=traj_est[:,3:],
#         timestamps=np.array(tstamps))

#     gt_file = os.path.join(args.datapath, 'groundtruth.txt')
#     traj_ref = file_interface.read_tum_trajectory_file(gt_file)

#     traj_ref, traj_est = sync.associate_trajectories(traj_ref, traj_est)
#     result = main_ape.ape(traj_ref, traj_est, est_name='traj', 
#         pose_relation=PoseRelation.translation_part, align=True, correct_scale=True)
    


#     print(result)

