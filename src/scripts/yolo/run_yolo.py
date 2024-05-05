import os
import json
import glob
import cv2
import sys
from ultralytics import YOLO

sessionbase = "/session"
if len(sys.argv) > 1:
    sessionbase = sys.argv[1]

# Load the YOLO model
model = YOLO("yolov8n.pt")

# Define the path to the RGB images folder
rgb_folder = sessionbase + "/rgb"

#if the folder is empty, then exit
if not os.listdir(rgb_folder):
    print('rgb folder is empty, exiting')
    exit()

# Load the ground truth data
with open(sessionbase + "/ground_truth.json") as f:
    gt_data = json.load(f)

os.makedirs(sessionbase + "/rgb_ground_truth", exist_ok=True)
os.makedirs(sessionbase + "/predictions", exist_ok=True)
os.makedirs(sessionbase +  "/gt_vs_decl", exist_ok=True)

# Initialize the dictionary to store the predictions
predictions = {
    "declJsonVersion": "0.0.1",
    "source": "MyAlgorithm",
    "fileUID": "example",
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

        # Get the frame number from the filename. file are named 0000000.png, 0000001.png, etc.
        frame_num = int(os.path.splitext(os.path.basename(filename))[0])

        #remove all leading zeros
        frame_num = str(frame_num).lstrip('0')

        if frame_num == '':
            frame_num = '0'
        
        frame_num = 'f' + frame_num

        #temporary disable ground truth annotation
        #this will be moved to a separate process
        # # Check if there are annotations for this frame
        # if frame_num in gt_data['frameAnnotations']:
        #     # Iterate over all the annotations for this frame
        #     for annotation in gt_data['frameAnnotations'][frame_num]['annotations']:
        #         # Get the bounding box coordinates
        #         bbox = annotation['shape']['data']

        #         # Draw the bounding box on the image
        #         print('drawing bbox', bbox)
        #         img_gt_decl = draw_xywh_on_rgb(img_gt_decl, [bbox], [annotation['class']], color=(0, 255, 0))
        # else:
        #     print('no annotations for frame', frame_num)

        # Save the annotated image to the output directory
        # cv2.imwrite(os.path.join(sessionbase + "/rgb_ground_truth', os.path.basename(filename)), img_gt_decl)


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

            # Find the index of the bounding box with the closest distance to the first bounding box in ground truth
            if bboxes and len(gt_data['frameAnnotations'][frame_num]['annotations']) > 0:
                first_gt_bbox = gt_data['frameAnnotations'][frame_num]['annotations'][0]['shape']['data']
                closest_bbox_index = min(range(len(bboxes)), key=lambda i: abs(bboxes[i][0] - first_gt_bbox[0]) + abs(bboxes[i][1] - first_gt_bbox[1]))
                # Keep only the closest bounding box, class, and confidence
                bboxes = [bboxes[closest_bbox_index]]
                classes = [classes[closest_bbox_index]]
                confidences = [confidences[closest_bbox_index]]
            else:
                print('no predictions for frame', frame_num)
                bboxes = []
                classes = []
                confidences = []
                continue

            img_decl = img.copy()

            #move the boxes by 1/2 width and height towards the upper left corner
            for box in bboxes:
                box[0] -= box[2]/2
                box[1] -= box[3]/2
            
            # Draw the bounding boxes on the image
            img_decl = draw_xywh_on_rgb(img_decl, bboxes, classes, color=(255, 0, 0))

            img_gt_decl = draw_xywh_on_rgb(img_gt_decl, bboxes, classes, color=(255, 0, 0))

            # Save the annotated image to the output directory
            cv2.imwrite(os.path.join(sessionbase + "/predictions", os.path.basename(filename)), img_decl)

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

        # Save the annotated image to the output directory
        # cv2.imwrite(os.path.join(sessionbase + "/gt_vs_decl", os.path.basename(filename)), img_gt_decl)

# Export the predictions to a JSON file
with open("/session/decl.json", "w") as f:
    json.dump(predictions, f)