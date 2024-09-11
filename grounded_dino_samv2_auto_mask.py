import matplotlib.pyplot as plt
import os
import cv2
import json
import torch
import numpy as np
import supervision as sv
import pycocotools.mask as mask_util
from pathlib import Path
from torchvision.ops import box_convert
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator
from grounding_dino.groundingdino.util.inference import load_model, load_image, predict


"""
Hyper parameters
"""
TEXT_PROMPT = "box."
IMG_PATH = "/home/gaurav/Operating_Room_Logestics/Perception_System/image4.png"
SAM2_CHECKPOINT = "./checkpoints/sam2_hiera_large.pt"
SAM2_MODEL_CONFIG = "sam2_hiera_l.yaml"
GROUNDING_DINO_CONFIG = "grounding_dino/groundingdino/config/GroundingDINO_SwinT_OGC.py"
GROUNDING_DINO_CHECKPOINT = "gdino_checkpoints/groundingdino_swint_ogc.pth"
BOX_THRESHOLD = 0.35
TEXT_THRESHOLD = 0.25
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
OUTPUT_DIR = Path("outputs/grounded_sam2_local_demo")
DUMP_JSON_RESULTS = True

# create output directory
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# build SAM2 image predictor
sam2_checkpoint = SAM2_CHECKPOINT
model_cfg = SAM2_MODEL_CONFIG
sam2_model = build_sam2(model_cfg, sam2_checkpoint, device=DEVICE)
sam2_predictor = SAM2AutomaticMaskGenerator(sam2_model)

# build grounding dino model
grounding_model = load_model(
    model_config_path=GROUNDING_DINO_CONFIG,
    model_checkpoint_path=GROUNDING_DINO_CHECKPOINT,
    device=DEVICE
)

# setup the input image and text prompt for SAM 2 and Grounding DINO
# VERY important: text queries need to be lowercased + end with a dot
text = TEXT_PROMPT
img_path = IMG_PATH

image_source, image = load_image(img_path)

# Get the height and width of the original image
h, w, _ = image_source.shape

# sam2_predictor.set_image(image_source)

# Predict boxes using Grounding DINO
boxes, confidences, labels = predict(
    model=grounding_model,
    image=image,
    caption=text,
    box_threshold=BOX_THRESHOLD,
    text_threshold=TEXT_THRESHOLD,
)

# Process the box prompt for SAM 2
boxes = boxes * torch.Tensor([w, h, w, h])  # Scale boxes to image dimensions
input_boxes = box_convert(boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy").numpy()

# Loop through each box, crop the image and apply SAM2 on the cropped region
cropped_results = []

# select the device for computation
if torch.cuda.is_available():
    device = torch.device("cuda")
elif torch.backends.mps.is_available():
    device = torch.device("mps")
else:
    device = torch.device("cpu")
print(f"using device: {device}")

if device.type == "cuda":
    # use bfloat16 for the entire notebook
    torch.autocast("cuda", dtype=torch.bfloat16).__enter__()
    # turn on tfloat32 for Ampere GPUs (https://pytorch.org/docs/stable/notes/cuda.html#tensorfloat-32-tf32-on-ampere-devices)
    if torch.cuda.get_device_properties(0).major >= 8:
        torch.backends.cuda.matmul.allow_tf32 = True
        torch.backends.cudnn.allow_tf32 = True
elif device.type == "mps":
    print(
        "\nSupport for MPS devices is preliminary. SAM 2 is trained with CUDA and might "
        "give numerically different outputs and sometimes degraded performance on MPS. "
        "See e.g. https://github.com/pytorch/pytorch/issues/84936 for a discussion."
    )


def show_anns(anns, borders=True):
    if len(anns) == 0:
        return
    sorted_anns = sorted(anns, key=(lambda x: x['area']), reverse=True)
    ax = plt.gca()
    ax.set_autoscale_on(False)

    img = np.ones((sorted_anns[0]['segmentation'].shape[0],
                  sorted_anns[0]['segmentation'].shape[1], 4))
    img[:, :, 3] = 0
    for ann in sorted_anns:
        m = ann['segmentation']
        color_mask = np.concatenate([np.random.random(3), [0.5]])
        img[m] = color_mask
        if borders:
            import cv2
            contours, _ = cv2.findContours(
                m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            # Try to smooth contours
            contours = [cv2.approxPolyDP(
                contour, epsilon=0.01, closed=True) for contour in contours]
            cv2.drawContours(img, contours, -1, (0, 0, 1, 0.4), thickness=1)

    ax.imshow(img)


for i, box in enumerate(input_boxes):
    # Crop the image based on the box coordinates (x_min, y_min, x_max, y_max)
    x_min, y_min, x_max, y_max = map(int, box)
    cropped_image = image_source[y_min:y_max, x_min:x_max]

    # Set the cropped image in SAM2
    # sam2_predictor.set_image(cropped_image)

    # print(cropped_image)
    # Predict the masks on the cropped image
    masks = sam2_predictor.generate(
        cropped_image
    )

    # print(masks)

    plt.figure(figsize=(20, 20))
    plt.imshow(cropped_image)
    show_anns(masks)
    plt.axis('off')
    plt.show()
    # Enable autocast for mixed precision
    # from torch.cuda.amp import autocast
    # with autocast(device_type='cuda', dtype=torch.float16):
    #     masks, scores, logits = sam2_predictor.generate(
    #         cropped_image  # Pass the cropped image
    #     )

    # Post-process masks to adjust the mask to the original image coordinates
    # if masks.ndim == 4:
    #     masks = masks.squeeze(1)

    # # Store results for each cropped region
    # cropped_results.append({
    #     "box": box.tolist(),
    #     "masks": masks,
    #     # "scores": scores,
    # })

    # # Save the cropped mask image for visualization
    # mask_annotator = sv.MaskAnnotator()
    # detections = sv.Detections(
    #     # (1, 4) for the cropped box
    #     xyxy=np.array([[x_min, y_min, x_max, y_max]]),
    #     mask=masks.astype(bool),  # Apply mask on the cropped region
    # )
    # annotated_frame = mask_annotator.annotate(
    #     scene=cropped_image.copy(), detections=detections)
    # cv2.imwrite(os.path.join(
    #     OUTPUT_DIR, f"grounded_sam2_annotated_image_cropped_{i}.jpg"), annotated_frame)

# Convert masks to RLE and save results as JSON


# def single_mask_to_rle(mask):
#     rle = mask_util.encode(
#         np.array(mask[:, :, None], order="F", dtype="uint8"))[0]
#     rle["counts"] = rle["counts"].decode("utf-8")
#     return rle


# if DUMP_JSON_RESULTS:
#     json_results = []
#     for result in cropped_results:
#         box = result['box']
#         masks = result['masks']
#         # scores = result['scores']

#         mask_rles = [single_mask_to_rle(mask) for mask in masks]

#         # Save the results in standard format
#         json_results.append({
#             "box": box,
#             "masks": mask_rles,
#             # "scores": scores.tolist(),
#         })

#     # Write the JSON file
#     with open(os.path.join(OUTPUT_DIR, "grounded_sam2_cropped_results.json"), "w") as f:
#         json.dump(json_results, f, indent=4)
