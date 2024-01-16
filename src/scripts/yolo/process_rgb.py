import os
import json
import glob
import cv2
from ultralytics import YOLO

# Function to save predictions as labeled copies of the images
def save_labeled_images(predictions, image_paths, output_folder):
    for image_path in image_paths:
        image = cv2.imread(image_path)
        image_name = os.path.basename(image_path)
        frame_title = os.path.splitext(image_name)[0]
        
        # Get the predictions for the current image
        frame_declarations = predictions["frameDeclarations"].get(frame_title, {})
        
        # Draw bounding boxes on the image
        for declaration in frame_declarations.get("declarations", []):
            cls = declaration["class"]
            conf = declaration["confidence"]
            box = declaration["shape"]["data"]
            x, y, w, h = box
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(image, f"{cls}: {conf:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        
        # Save the labeled image
        labeled_image_path = os.path.join(output_folder, image_name)
        cv2.imwrite(labeled_image_path, image)

# Function to save ground truth images with boxes/labels
def save_ground_truth_images(ground_truth, image_paths, output_folder):
    for image_path in image_paths:
        image = cv2.imread(image_path)
        image_name = os.path.basename(image_path)
        frame_title = os.path.splitext(image_name)[0]
        
        # Get the ground truth annotations for the current image
        frame_annotations = ground_truth["frameAnnotations"].get(frame_title, {})
        
        # Draw bounding boxes on the image
        for annotation in frame_annotations.get("annotations", []):
            cls = annotation["class"]
            box = annotation["shape"]["data"]
            x, y, w, h = box
            cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(image, cls, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
        
        # Save the labeled image
        labeled_image_path = os.path.join(output_folder, image_name)
        cv2.imwrite(labeled_image_path, image)

# Path to the RGB images folder
rgb_folder = "/session/rgb"

# Path to the predictions folder
predictions_folder = "/session/predictions"

# Path to the ground truth folder
ground_truth_folder = "/session/gt"

# Path to the predictions.json file
predictions_file = "/session/predictions.json"

# Path to the ground_truth.json file
ground_truth_file = "/session/gt/ground_truth.json"

# Load the YOLO model
yolo = YOLO()

# Get the list of PNG files in the RGB folder
image_paths = glob.glob(os.path.join(rgb_folder, "*.png"))

# Create the predictions dictionary
predictions = {
    "declJsonVersion": "0.0.1",
    "source": "MyAlgorithm",
    "fileUID": "example",
    "frameDeclarations": {}
}

# Process each image
image_num = 0
for image_path in image_paths:
    image = cv2.imread(image_path)
    image_name = os.path.basename(image_path)
    frame_title = os.path.splitext(image_name)[0]
    
   # Run the YOLO predictor on the image
    results = yolo.predict(image)

    for result in results:
        # Extract the bounding boxes, classes, and confidences from the results
        bboxes = result.boxes.xywh.tolist() if result.boxes is not None else []
        classes = result.boxes.cls.tolist() if result.names is not None else []
        classes = [result.names[i] for i in classes]
        confidences = result.boxes.conf.tolist() if result.boxes is not None else []

        # Create the frame declaration for the current image
        frame_declaration = {
            "declarations": [
                {
                    "class": cls,
                    "confidence": conf,
                    "shape": {
                        "data": box,
                        "type": "bbox_xywh"
                    }
                } for box, cls, conf in zip(bboxes, classes, confidences)
            ]
        }
        
        # Add the frame declaration to the predictions dictionary
        predictions["frameDeclarations"][frame_title] = frame_declaration

# Save the labeled images
os.makedirs(predictions_folder, exist_ok=True)
save_labeled_images(predictions, image_paths, predictions_folder)

# Load the ground truth data
with open(ground_truth_file, "r") as f:
    ground_truth = json.load(f)

# Create the rgb_gt folder
rgb_gt_folder = os.path.join("/session", "rgb_gt")
os.makedirs(rgb_gt_folder, exist_ok=True)

# Save the ground truth images with boxes/labels
save_ground_truth_images(ground_truth, image_paths, rgb_gt_folder)

# Create the gt_vs_pred folder
gt_vs_pred_folder = os.path.join("/session", "gt_vs_pred")
os.makedirs(gt_vs_pred_folder, exist_ok=True)

# Save the labeled images with ground truth and predictions
save_labeled_images(ground_truth, image_paths, gt_vs_pred_folder)
save_labeled_images(predictions, image_paths, gt_vs_pred_folder)

# Save the predictions.json file
with open(predictions_file, "w") as f:
    json.dump(predictions, f)
