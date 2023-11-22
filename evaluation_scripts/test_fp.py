import sys
sys.path.append('droid_slam')

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


import torch.nn.functional as F
from droid import Droid


def show_image(image):
#     print(image.shape)
    image = image.permute(1, 2, 0).cpu().numpy()
    cv2.imshow('image', image / 255.0)
    cv2.waitKey(1)

def image_stream(datapath, calib, image_size=[320, 512]):
    """ image generator """

    calib = np.loadtxt(calib, delimiter=" ")
    fx, fy, cx, cy = calib[:4]

    K = np.eye(3)
    K[0,0] = fx
    K[0,2] = cx
    K[1,1] = fy
    K[1,2] = cy

    # read all png images in folder
    image_dir = os.path.join(datapath, 'left')
    images_list = sorted(glob.glob(os.path.join(image_dir, '*.png')))
      
    for t, imfile in enumerate(images_list):
        image = cv2.imread(os.path.join(image_dir, imfile))
        if len(calib) > 4:
            image = cv2.undistort(image, K, calib[4:])

        h0, w0, _ = image.shape
        h1 = int(h0 * np.sqrt((384 * 512) / (h0 * w0)))
        w1 = int(w0 * np.sqrt((384 * 512) / (h0 * w0)))

        image = cv2.resize(image, (w1, h1))
        image = image[:h1-h1%8, :w1-w1%8]
      #   print(image.shape)
        image = torch.as_tensor(image).permute(2, 0, 1)
      #   print(image.shape)

        intrinsics = torch.as_tensor([fx, fy, cx, cy])
        intrinsics[0::2] *= (w1 / w0)
        intrinsics[1::2] *= (h1 / h0)

        yield t, image[None], intrinsics

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--datapath")
    parser.add_argument("--weights", default="droid.pth")
    parser.add_argument("--buffer", type=int, default=512)
    parser.add_argument("--image_size", default=[240, 320])
    parser.add_argument("--disable_vis", action="store_true")
    parser.add_argument("--calib")
    
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
    parser.add_argument("--timestamp", type=str, default="tstamps.txt")
    parser.add_argument("--reconstruction_path", help="path to saved reconstruction")
    parser.add_argument("--upsample", action="store_true")

    args = parser.parse_args()

    args.stereo = False
    torch.multiprocessing.set_start_method('spawn')

    print("Running evaluation on {}".format(args.datapath))
    print(args)

    if args.reconstruction_path is not None:
        args.upsample = True

    droid = Droid(args)
    time.sleep(5)

    tstamps = []
    tstamps = np.loadtxt(os.path.join(args.datapath, args.timestamp))
    for (t, image, intrinsics) in tqdm(image_stream(args.datapath, args.calib)):
      #   print(image.shape)
        if not args.disable_vis:
            show_image(image[0])
        droid.track(t, image, intrinsics=intrinsics)


    traj_est = droid.terminate(image_stream(args.datapath, args.calib))

    ### run evaluation ###
    print("#"*20 + " Results...")

    traj_est = PoseTrajectory3D(
        positions_xyz=traj_est[:,:3],
        orientations_quat_wxyz=traj_est[:,3:],
        timestamps=np.array(tstamps))


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

