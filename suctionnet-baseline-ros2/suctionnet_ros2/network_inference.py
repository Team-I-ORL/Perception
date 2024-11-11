from neural_network import network
import torch.nn.functional as F
import cv2
import torch
import numpy as np
import open3d as o3d
import torch.nn as nn
from neural_network import get_suction_from_heatmap
from .util import CameraInfo
from .util import SuctionNetUtils as SNU

class SuctionNetInferencer:
    def __init__(self, ckpt_path):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.net = network.deeplabv3plus_resnet101(num_classes=2, output_stride=16)
        self.net = nn.DataParallel(self.net)
        self.net.to(self.device)
        checkpoint = torch.load(ckpt_path)
        self.net.load_state_dict(checkpoint['model_state_dict'])

        # self.camera_info = CameraInfo(1280, 720, 525.8810348926615, 527.8853163315471, 321.0291284324178, 228.7422250324759, 1000)
        self.camera_info = CameraInfo(640, 480, 525.8810348926615, 527.8853163315471, 321.0291284324178, 228.7422250324759, 1000)
        
        self.net.eval()

    def infer(self, rgb_img : np.ndarray, depth_img : np.ndarray, seg_mask = None):
        assert rgb_img.shape[:2] == depth_img.shape[:2]
        assert len(depth_img.shape) == 2
        rgb = torch.from_numpy(rgb_img)
        depth = torch.from_numpy(depth_img).clamp(0, 1)
        rgbd = torch.cat([rgb, depth.unsqueeze(-1)], dim=-1).unsqueeze(0)
        rgbd = rgbd.permute(0, 3, 1, 2)
        rgbd = rgbd.to(self.device)
        with torch.no_grad():
            pred = self.net(rgbd).clamp(0, 1)
        heatmap = (pred[0, 0] * pred[0, 1]).cpu().unsqueeze(0).unsqueeze(0)

        k_size = 15
        kernel = SNU.uniform_kernel(k_size)
        kernel = torch.from_numpy(kernel).unsqueeze(0).unsqueeze(0)
        heatmap = F.conv2d(heatmap, kernel, padding=(kernel.shape[2] // 2, kernel.shape[3] // 2)).squeeze().numpy()

        suctions, idx0, idx1 = get_suction_from_heatmap(depth.numpy(), heatmap, self.camera_info)
        print("suction shape before filtering: ", suctions.shape)

        if seg_mask is not None:
            suctions, idx0, idx1 = SNU.filter_suctions(suctions, idx0, idx1, seg_mask)
        
        print("suction shape after filtering: ", suctions.shape)

        suction_scores = suctions[:, 0]
        suction_normals = suctions[:, 1:4]
        suction_points = suctions[:, 4:7]

        SNU.visualize_heatmap(heatmap, rgb_img*255, idx0, idx1)
        # visualize_suctions(suction_normals, suction_points)

if __name__ == "__main__":
    ckpt_path = "/home/jinkai/Downloads/test_suctnet/models/realsense-deeplabplus-RGBD"
    inferencer = SuctionNetInferencer(ckpt_path)

    depth_dir = "/home/jinkai/Downloads/data/depth_image/first_scene/frame0000.jpg"
    rgb_dir = "/home/jinkai/Downloads/data/color_image/first_scene/frame0000.jpg"
    seg_dir = "/home/jinkai/Downloads/data/seg_masks/first_scene.png"
    rgb_img = cv2.imread(rgb_dir).astype(np.float32) / 255.0
    depth_img = cv2.imread(depth_dir, cv2.IMREAD_GRAYSCALE).astype(np.float32) / 1000.0
    seg_mask = cv2.imread(seg_dir, cv2.IMREAD_GRAYSCALE).astype(bool)

    # rgb_dir = "/home/jinkai/Downloads/test_suctnet/scenes/scene_0130/realsense/rgb/0000.png"
    # depth_dir = "/home/jinkai/Downloads/test_suctnet/scenes/scene_0130/realsense/depth/0000.png"
    # seg_dir = "/home/jinkai/Downloads/test_suctnet/scenes/scene_0130/realsense/label/0000.png"
    # rgb_img = cv2.imread(rgb_dir).astype(np.float32) / 255.0
    # depth_img = cv2.imread(depth_dir, cv2.IMREAD_UNCHANGED).astype(np.float32) / 1000.0
    # seg_mask = cv2.imread(seg_dir, cv2.IMREAD_UNCHANGED).astype(bool)
    
    inferencer.infer(rgb_img, depth_img, seg_mask)




