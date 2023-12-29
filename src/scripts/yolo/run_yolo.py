import os
import json
import glob
import cv2
from ultralytics import YOLO

# Load the YOLO model
model = YOLO("yolov8n.pt")

# Define the path to the RGB images folder
rgb_folder = "/session/rgb"

#if the folder is empty, then exit
if not os.listdir(rgb_folder):
    print('rgb folder is empty, exiting')
    exit()

# Load the ground truth data
with open('/session/gt/ground_truth.json') as f:
    gt_data = json.load(f)

# Create the output directories if they don't exist, with full permissions
os.makedirs('/session/rgb_ground_truth', exist_ok=True)
os.makedirs('/session/predictions', exist_ok=True)
os.makedirs('/session/gt_vs_decl', exist_ok=True)

# Initialize the dictionary to store the predictions
predictions = {
    "collection": "MyCollection",
    "fileUID": "example",
    "startTime": "2022-1-5T13:21:05.431000",
    "stopTime": "2022-1-5T13:22:18.725000",
    "nFrames": 0,
    "frameDeclarations": {}
}

def draw_xywh_on_rgb(image, boxes, classes, color=(0, 255, 0), thickness=2):
    """
    Draw the bounding boxes on the RGB images
    """
    # Loop through each bounding box
    for box, cls in zip(boxes, classes):
        # Extract the bounding box coordinates
        x, y, w, h = box

        # Draw the bounding box rectangle and label on the image
        cv2.rectangle(image, (int(x), int(y)), (int(x + w), int(y + h)), color, thickness)
        cv2.putText(image, cls, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness)

    return image

# Loop through each image in the RGB folder
# go from earliest to latest created
files=glob.glob(rgb_folder + '/*.png')
files.sort(key=os.path.getmtime)
for filename in files:
    if filename.endswith(".png"):
        # Load the image
        img = cv2.imread(filename)
        img_gt_decl = img.copy()

        # Predict on the image
        results = model(img)

        # Get the frame number from the filename
        frame_num = 'f' + os.path.basename(filename).split('.')[0]

        # Check if there are annotations for this frame
        if frame_num in gt_data['frameAnnotations']:
            # Iterate over all the annotations for this frame
            for annotation in gt_data['frameAnnotations'][frame_num]['annotations']:
                # Get the bounding box coordinates
                bbox = annotation['shape']['data']

                # Draw the bounding box on the image
                img_gt_decl = draw_xywh_on_rgb(img_gt_decl, [bbox], [annotation['class']], color=(0, 255, 0))

        # Save the annotated image to the output directory
        cv2.imwrite(os.path.join('/session/rgb_ground_truth', os.path.basename(filename)), img_gt_decl)


        for result in results:
            # Extract the bounding boxes, classes, and confidences from the results
            bboxes = result.boxes.xywh if result.boxes is not None else []
            classes = result.boxes.cls
            confidences = result.boxes.conf if result.boxes is not None else []

            #bboxes and classes are Tensors, convert to lists
            bboxes = bboxes.tolist()
            classes = classes.tolist()
            confidences = confidences.tolist()

            classes = [result.names[i] for i in classes]

            img_decl = img.copy()
            img_decl = draw_xywh_on_rgb(img_decl, bboxes, classes, color=(255, 0, 0))

            img_gt_decl = draw_xywh_on_rgb(img_gt_decl, bboxes, classes, color=(255, 0, 0))

            # Save the annotated image to the output directory
            cv2.imwrite(os.path.join('/session/predictions', os.path.basename(filename)), img_decl)

            # Add the predictions to the dictionary
            frame_id = os.path.splitext(os.path.basename(filename))[0]
            frame_title = "f" + frame_id
            predictions["frameDeclarations"][frame_title] = {
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

        # Increment the frame count
        predictions["nFrames"] += 1

        # Save the annotated image to the output directory
        cv2.imwrite(os.path.join('/session/gt_vs_decl', os.path.basename(filename)), img_gt_decl)

# Export the predictions to a JSON file
with open("/session/decl.json", "w") as f:
    json.dump(predictions, f)