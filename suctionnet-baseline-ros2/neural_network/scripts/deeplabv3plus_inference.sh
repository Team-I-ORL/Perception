CUDA_VISIBLE_DEVICES=0 python ../inference.py --model deeplabv3plus_resnet101 \
--checkpoint_path /home/jinkai/Downloads/test_suctnet/models/realsense-deeplabplus-RGBD \
--split test_similiar \
--camera realsense \
--dataset_root /home/jinkai/Downloads/test_suctnet \
--save_dir /home/jinkai/Downloads/test_suctnet/net \
--save_visu

