# Test script for YOLO inference and drawing bounding boxes
from ultralytics import YOLO
import cv2

model = YOLO("yolov10s.pt")

results = model("image.png")

# Draw ALL detection bounding boxes and confidences
# Replace with custom draw or inference script to filter classes
annotated = results[0].plot()   

# Display image
cv2.imshow("annotated", annotated)
cv2.waitKey(0)
cv2.destroyAllWindows()