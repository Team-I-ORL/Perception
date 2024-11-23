import os
import cv2
import time
import argparse
import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image
import scipy.io as scio
import sys
from normal_std.inference import estimate_suction
from .util import CameraInfo
from .util import SuctionNetUtils as SNU
from scipy.cluster.hierarchy import linkage, fcluster

class NormStdInferencer:
    def __init__(self):
        # PrimeSense camera info
        self.camera_info = CameraInfo(640, 480, 525.8810348926615, 527.8853163315471, 321.0291284324178, 228.7422250324759, 1000)
    
    # Cluster the suction directions and translations in case segmask covers multiple objects
    def cluster_from_suction(self, suction_directions, suction_translations):
        Z = linkage(suction_translations, 'ward')
        distances = Z[:, 2]
        diff = np.diff(distances)
        max_gap_index = np.argmax(diff)
        threshold = (distances[max_gap_index] + distances[max_gap_index + 1]) / 2
        clusters = fcluster(Z, threshold, criterion='distance')
        
        clustered_suction_directions = []
        clustered_suction_translations = []

        for cluster_id in np.unique(clusters):
            cluster_indices = np.where(clusters == cluster_id)[0]
            cluster_directions = suction_directions[cluster_indices]
            cluster_translations = suction_translations[cluster_indices]
            clustered_suction_directions.append(cluster_directions)
            clustered_suction_translations.append(cluster_translations)

        return clustered_suction_directions, clustered_suction_translations

    def infer(self, rgb_img : np.ndarray, depth_img : np.ndarray, seg_mask=None, use_cluster=False, centerness_weight=0.5):
        assert rgb_img.shape[:2] == depth_img.shape[:2]
        assert rgb_img.shape[:2] == seg_mask.shape[:2]
        assert len(depth_img.shape) == 2
        assert len(seg_mask.shape) == 2
        heatmap, normals, point_cloud = estimate_suction(depth_img, seg_mask, self.camera_info)

        # Visualize heatmap
        k_size = 15
        kernel = SNU.uniform_kernel(k_size)
        kernel = torch.from_numpy(kernel).unsqueeze(0).unsqueeze(0)
        # print('kernel:', kernel.shape)
        heatmap = np.pad(heatmap, k_size//2)
        heatmap = torch.from_numpy(heatmap).unsqueeze(0).unsqueeze(0)
        heatmap = F.conv2d(heatmap, kernel).squeeze().numpy()

        # Create centerness mask
        centerness_mask = np.zeros_like(seg_mask, dtype=np.float32)
        for region in np.unique(seg_mask):
            if region == 0:
                continue
            region_mask = (seg_mask == region)
            region_indices = np.argwhere(region_mask)
            center = np.mean(region_indices, axis=0).astype(int)
            distances = np.linalg.norm(region_indices - center, axis=1)
            max_distance = np.max(distances)
            centerness = 1 - (distances / max_distance)
            centerness_mask[region_mask] = centerness

        heatmap_avg = np.mean(heatmap)
        centerness_avg = np.mean(centerness_mask)
        centerness_mask = centerness_mask / centerness_avg * centerness_weight
        heatmap = heatmap / heatmap_avg

        heatmap = np.dot(heatmap, centerness_weight)

        suction_scores, idx0, idx1 = SNU.grid_sample(heatmap, down_rate=10, topk=20)
        
        if seg_mask is not None:
            suctions, idx0, idx1 = SNU.filter_suctions(suction_scores, idx0, idx1, seg_mask)

        suction_directions = normals[idx0, idx1, :]
        suction_translations = point_cloud[idx0, idx1, :]
        # SNU.visualize_heatmap(heatmap, rgb_img, idx0, idx1)
        # SNU.visualize_suctions(suction_directions, suction_translations)
        
        if use_cluster:
            clu_suction_directions, clu_suction_translations = self.cluster_from_suction(suction_directions, suction_translations)
            print('Num of clusters:', len(clu_suction_directions))

            sampled_index = np.random.choice(range(len(clu_suction_directions)))

            suction_directions = clu_suction_directions[sampled_index]
            suction_translations = clu_suction_translations[sampled_index]
            # print('Sampled suction_directions:', suction_directions)
            # print('Sampled suction_translations:', suction_translations)
        
        # average suction direction and translation after filtering outliers
        suction_direction = np.mean(suction_directions, axis=0)
        suction_direction = suction_direction / np.linalg.norm(suction_direction)
        suction_translation = np.mean(suction_translations, axis=0)
        suction_quat = SNU.unit_vect_to_quat(suction_direction)
        
        print('suction_quat:', suction_quat)
        print('suction_translation:', suction_translation)
        print('suction_direction:', suction_direction)

        # Find points with min and max X coordinates
        min_x_idx = np.argmin(suction_translations[:, 0])
        max_x_idx = np.argmax(suction_translations[:, 0])
        min_x_point = suction_translations[min_x_idx]
        max_x_point = suction_translations[max_x_idx]
        # print('Point with minimum X:', min_x_point)
        # print('Point with maximum X:', max_x_point)
        # Find points with min and max Y coordinates
        min_y_idx = np.argmin(suction_translations[:, 1])
        max_y_idx = np.argmax(suction_translations[:, 1])
        min_y_point = suction_translations[min_y_idx]
        max_y_point = suction_translations[max_y_idx]
        # print('Point with minimum Y:', min_y_point)
        # print('Point with maximum Y:', max_y_point)

        # Calculate differences in X and Y
        x_diff = max_x_point[0] - min_x_point[0]
        y_diff = max_y_point[1] - min_y_point[1]
        # print('X difference:', x_diff)
        # print('Y difference:', y_diff)

        return suction_quat, suction_translation , (x_diff, y_diff)

if __name__ == "__main__":
    inferencer = NormStdInferencer()
    # depth_dir = "/home/jinkai/Downloads/data/depth_image/first_scene/frame0000.jpg"
    # rgb_dir = "/home/jinkai/Downloads/data/color_image/first_scene/frame0000.jpg"
    seg_dir = "/home/jinkai/Downloads/data/seg_masks/first_scene.png"
    depth_dir = "/home/jinkai/Downloads/Test_Images/Depth/image_50.png"
    rgb_dir = "/home/jinkai/Downloads/Test_Images/RGB/image_50.png"
    seg_dir = "/home/jinkai/Downloads/Test_Images/seg_mask/image_50_2.png"
    rgb_img = cv2.imread(rgb_dir)
    depth_img = cv2.imread(depth_dir, cv2.IMREAD_GRAYSCALE).astype(np.float32)/100
    dummy_seg_mask = np.ones_like(depth_img, dtype=bool)
    seg_mask = cv2.imread(seg_dir, cv2.IMREAD_GRAYSCALE).astype(bool)
    
    inferencer.infer(rgb_img, depth_img, seg_mask)