import json
import sys

def calculate_iou(box1, box2):
    """Calculate Intersection over Union (IoU) of two bounding boxes."""
    x1, y1, w1, h1 = box1
    x2, y2, w2, h2 = box2

    xi1 = max(x1, x2)
    yi1 = max(y1, y2)
    xi2 = min(x1 + w1, x2 + w2)
    yi2 = min(y1 + h1, y2 + h2)
    inter_area = max(xi2 - xi1, 0) * max(yi2 - yi1, 0)

    box1_area = w1 * h1
    box2_area = w2 * h2
    union_area = box1_area + box2_area - inter_area

    return inter_area / union_area

# Load YOLO predictions and ground truth data
with open('/home/mindful/HeadlessUnrealFlightAgent/bags/2023-12-18_16-12-19/1/decl.json', 'r') as f:
    yolo_data = json.load(f)

with open('/home/mindful/HeadlessUnrealFlightAgent/bags/2023-12-18_16-12-19/1/gt/ground_truth.json', 'r') as f:
    gt_data = json.load(f)

if len(sys.argv) > 2:
    with open(sys.argv[1], 'r') as f:
        yolo_data = json.load(f)
    
    with open(sys.argv[2], 'r') as f:
        gt_data = json.load(f)

# Score the results
iou_scores = []
for frame in yolo_data['frameDeclarations']:
    if frame in gt_data['frameAnnotations']:
        yolo_box = yolo_data['frameDeclarations'][frame]['declarations'][0]['shape']['data']
        gt_box = gt_data['frameAnnotations'][frame]['annotations'][0]['shape']['data']
        iou = calculate_iou(yolo_box, gt_box)
        iou_scores.append(iou)

# Print average IoU score
sum = sum(iou_scores)
score = 0
if sum == 0:
    print('Average IoU score: 0')
else:
    score = sum / len(iou_scores)
    print('Average IoU score:', score)

#save the txt with the score in the same folder as the yolo json file
if len(sys.argv) > 2:
    with open(sys.argv[1].replace(".json", ".txt"), 'w') as f:
        f.write(str(score))