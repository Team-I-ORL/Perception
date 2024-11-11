# python inference.py --split test_seen \
# --camera kinect \
# --save_root /DATA2/Benchmark/suction/inference_results/normals_std \
# --dataset_root /DATA2/Benchmark/graspnet \
# --save_visu

# python inference.py --split test_similiar \
# --camera kinect \
# --save_root /DATA2/Benchmark/suction/inference_results/normals_std \
# --dataset_root /DATA2/Benchmark/graspnet \
# --save_visu

# python inference.py --split test_novel \
# --camera kinect \
# --save_root /DATA2/Benchmark/suction/inference_results/normals_std \
# --dataset_root /DATA2/Benchmark/graspnet \
# --save_visu

# python inference.py --split test_seen \
# --camera realsense \
# --save_root /DATA2/Benchmark/suction/inference_results/normals_std \
# --dataset_root /DATA2/Benchmark/graspnet \
# --save_visu

# python inference.py --split test_similiar \
# --camera realsense \
# --save_root /DATA2/Benchmark/suction/inference_results/normals_std \
# --dataset_root /DATA2/Benchmark/graspnet \
# --save_visu

python inference.py --split test_similiar \
--camera realsense \
--save_root /home/jinkai/Downloads/test_suctnet/normal_std \
--dataset_root /home/jinkai/Downloads/test_suctnet \
--save_visu
