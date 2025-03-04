
from ultralytics import YOLO
import cv2
import time
import matplotlib.pyplot as plt

class bottle_detector:
    def __init__(self):
        #self.model = YOLO("runs/detect/train/weights/best.onnx")  # Load trained model
        self.model = YOLO("runs/detect/train/weights/best.pt")  # Load trained model

    def get_x_pct(self, image):
        image_height, image_width, _ = image.shape  # Get height and width

        results = self.model.predict(image, save=False, conf=0.5)
        result = results[0]

        if len(result.boxes) == 0:
            print("Nothing detected")
            return None
        
        class_indices = result.boxes.cls.cpu().numpy().astype(int)  # Get class indices
        class_names = [self.model.names[idx] for idx in class_indices]  # Map indices to class names

        for i, class_name in enumerate(class_names):
            if class_name == "WaterBottle":
                x_center, y_center, width, height = result.boxes.xywh[i].tolist()  # Get corresponding detection
                x_pct = x_center / float(image_width)
                y_pct = y_center / float(image_height)
                return x_pct, y_pct  # Return first detected water bottle center percentage
            
        print("No water bottle detected.")
        return None


inst = bottle_detector()
image = cv2.imread("it.jpg")
x_pct, y_pct = inst.get_x_pct(image)
print(f"WaterBottle detected at: ({x_pct:.2f}, {y_pct:.2f})")