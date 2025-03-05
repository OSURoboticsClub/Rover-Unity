from ultralytics import YOLO
import cv2
import numpy as np
import time
import matplotlib.pyplot as plt

class bottle_detector:
    def __init__(self):
        self.model = YOLO("best.pt")  # Load trained model
        #self.model = YOLO("~/bottle_detector.pt")  # Load trained model

    def get_bottle(self, image):
        image_height, image_width, _ = image.shape  # Get height and width
        image_diagonal = np.sqrt(image_width**2 + image_height**2)  # Compute image diagonal distance

        results = self.model.predict(image, save=False, conf=0.5)
        result = results[0]

        if len(result.boxes) == 0:
            print("No water bottle detected.")
            return None
        
        class_indices = result.boxes.cls.cpu().numpy().astype(int)  # Get class indices
        class_names = [self.model.names[idx] for idx in class_indices]  # Map indices to class names

        for i, class_name in enumerate(class_names):
            if class_name == "WaterBottle":
                x_center, y_center, width, height = result.boxes.xywh[i].tolist()  # Get detection info
                
                # Calculate x and y percentages
                x_pct = x_center / float(image_width)
                y_pct = y_center / float(image_height)

                # Get top-right and bottom-left corner coordinates
                x1, y1, x2, y2 = result.boxes.xyxy[i].tolist()
                top_right = (x2, y1)
                bottom_left = (x1, y2)

                # Compute diagonal distance between top-right and bottom-left
                diag_distance = np.sqrt((top_right[0] - bottom_left[0])**2 + (top_right[1] - bottom_left[1])**2)
                diag_pct = diag_distance / image_diagonal  # Convert to percentage of image diagonal
                # result.show()
                return x_pct, diag_pct 
            
        print("Something was detected but it wasn't a bottle")
        return None, None


inst = bottle_detector()
image = cv2.imread("it.jpg")
result = inst.get_bottle(image)

if result:
    x_pct, y_pct, diag_pct = result
    print(f"WaterBottle detected at: {x_pct:.2f}, Diagonal %: {diag_pct:.2f}")
