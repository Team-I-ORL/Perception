import numpy as np
import cv2
from scipy.stats import zscore
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

class CameraInfo():
    def __init__(self, width, height, fx, fy, cx, cy, scale):
        self.width = width
        self.height = height
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.scale = scale

class SuctionNetUtils():
    def __init__(self):
        pass
    def uniform_kernel(kernel_size):
        kernel = np.ones((kernel_size, kernel_size), dtype=np.float32)
        # center = kernel_size // 2
        kernel = kernel / kernel_size**2

        return kernel

    def grid_sample(pred_score_map, down_rate=20, topk=512):
        num_row = pred_score_map.shape[0] // down_rate
        num_col = pred_score_map.shape[1] // down_rate

        idx_list = []
        for i in range(num_row):
            for j in range(num_col):
                pred_score_grid = pred_score_map[i*down_rate:(i+1)*down_rate, j*down_rate:(j+1)*down_rate]
                
                max_idx = np.argmax(pred_score_grid)
                max_idx = np.array([max_idx // down_rate, max_idx % down_rate]).astype(np.int32)
                
                max_idx[0] += i*down_rate
                max_idx[1] += j*down_rate
                idx_list.append(max_idx[np.newaxis, ...])
        
        idx = np.concatenate(idx_list, axis=0)
        suction_scores = pred_score_map[idx[:, 0], idx[:, 1]]
        sort_idx = np.argsort(suction_scores)
        sort_idx = sort_idx[::-1]

        sort_idx_topk = sort_idx[:topk]

        suction_scores_topk = suction_scores[sort_idx_topk]
        idx0_topk = idx[:, 0][sort_idx_topk]
        idx1_topk = idx[:, 1][sort_idx_topk]

        return suction_scores_topk, idx0_topk, idx1_topk

    def create_point_cloud_from_depth_image(depth, camera, organized=True):
        assert(depth.shape[0] == camera.height and depth.shape[1] == camera.width)
        xmap = np.arange(camera.width)
        ymap = np.arange(camera.height)
        xmap, ymap = np.meshgrid(xmap, ymap)
        points_z = depth
        points_x = (xmap - camera.cx) * points_z / camera.fx
        points_y = (ymap - camera.cy) * points_z / camera.fy
        cloud = np.stack([points_x, points_y, points_z], axis=-1)
        if not organized:
            cloud = cloud.reshape([-1, 3])
        return cloud

    def filter_suctions(suctions, idx0, idx1, seg_mask):
        mask_indices = np.where(seg_mask)
        valid_indices = np.array([idx for idx in zip(idx0, idx1) if (idx[0] in mask_indices[0] and idx[1] in mask_indices[1])])
        valid_mask = np.isin(np.array(list(zip(idx0, idx1))), valid_indices).all(axis=1)
        suctions = suctions[valid_mask]
        idx0 = idx0[valid_mask]
        idx1 = idx1[valid_mask]
        return suctions, idx0, idx1
        
    def visualize_heatmap(heatmap, rgb_img, idx0, idx1, rgb_blend_ratio=0.5):
        score_image = heatmap
        score_image *= 255
        score_image = score_image.clip(0, 255)
        score_image = score_image.astype(np.uint8)
        score_image = cv2.applyColorMap(score_image, cv2.COLORMAP_RAINBOW)
        rgb_image = rgb_blend_ratio * rgb_img + (1-rgb_blend_ratio) * score_image
        rgb_image = rgb_image.astype(np.uint8)

        # Plot dots on the idx0, idx1 positions
        for i in range(len(idx0)):
            cv2.circle(rgb_image, (idx1[i], idx0[i]), 5, (0, 255, 0), -1)
        
        avg_idx0 = int(np.mean(idx0))
        avg_idx1 = int(np.mean(idx1))
        cv2.circle(rgb_image, (avg_idx1, avg_idx0), 15, (255, 0, 0), -1) 
        
        # Display the image using Matplotlib
        plt.imshow(rgb_image)
        plt.title('Heatmap Visualization')
        plt.axis('off')  # Hide the axis
        plt.show()

    def visualize_suctions(suction_directions, suction_translations):
        # Extract the components of the suction directions and translations
        trans_x = suction_translations[:, 0]
        trans_y = suction_translations[:, 1]
        trans_z = suction_translations[:, 2]
        avg_trans_x, avg_trans_y, avg_trans_z = np.mean(trans_x), np.mean(trans_y), np.mean(trans_z)

        dir_x = suction_directions[:, 0]
        dir_y = suction_directions[:, 1]
        dir_z = suction_directions[:, 2]
        avg_dir_x, avg_dir_y, avg_dir_z = np.mean(dir_x), np.mean(dir_y), np.mean(dir_z)

        # Create a 3D scatter plot for the translations
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(trans_x, trans_y, trans_z, color='blue', label='Translations')

        # Use the quiver function to plot the directions as vectors
        ax.quiver(trans_x, trans_y, trans_z, dir_x, dir_y, dir_z, length=0.02, color='red', label='Directions')

        ax.scatter(avg_trans_x, avg_trans_y, avg_trans_z, color='green', label='Average Translation')
        ax.quiver(avg_trans_x, avg_trans_y, avg_trans_z, avg_dir_x, avg_dir_y, avg_dir_z, length=0.1, color='purple', label='Average Direction')


        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Suction Directions and Translations')
        ax.legend()

        # Set equal aspect ratio
        max_range = np.array([trans_x.max()-trans_x.min(), trans_y.max()-trans_y.min(), trans_z.max()-trans_z.min()]).max() / 2.0
        mid_x = (trans_x.max()+trans_x.min()) * 0.5
        mid_y = (trans_y.max()+trans_y.min()) * 0.5
        mid_z = (trans_z.max()+trans_z.min()) * 0.5
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
        
        plt.show()

    def remove_outliers(suction_directions, threshold):
        z_scores = zscore(suction_directions, axis=0)
        mask = np.all(np.abs(z_scores) < threshold, axis=1)
        filtered_suction_directions = suction_directions[mask]
        
        return filtered_suction_directions

    def unit_vect_to_quat(unit_vect):
        unit_vect = unit_vect / np.linalg.norm(unit_vect)
        ref_vect = np.array([1, 0, 0])
        cross = np.cross(unit_vect, ref_vect)
        dot = np.dot(unit_vect, ref_vect)


        w = dot + 1
        x = cross[0]
        y = cross[1]
        z = cross[2]

        l = np.sqrt(w*w+x*x+y*y+z*z)

        return (x/l,y/l,z/l,w/l)