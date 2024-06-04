from ultralytics import YOLO

hensuu = YOLO(model="yolov8n.pt")

result = hensuu("potato.jpg", save=True)