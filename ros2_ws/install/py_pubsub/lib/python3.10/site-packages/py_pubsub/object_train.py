from ultralytics import YOLO


if __name__ == "__main__":
    # Load a YOLOv8 model (pre-trained)
    model = YOLO("yolov8s.pt")  # 'n' is nano, you can also use 's', 'm', 'l', or 'x'

    # Train the model
    results = model.train(
        data="C:\\Projects\\dataset\\data.yaml",  # Path to your dataset.yaml file
        epochs=100,  # Number of training epochs
        patience=10,
        imgsz=640,  # Image size
        batch=16,  # Batch size (adjust based on GPU memory)
        device="cuda"  # Use 'cpu' if you don't have a GPU
    )
    model.export(format="onnx", imgsz=640, opset=12)
    metrics = model.val()
    print(metrics)



