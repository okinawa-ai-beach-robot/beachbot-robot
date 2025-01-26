import cv2
import torch
from PIL import Image

# Model
model = torch.hub.load("ultralytics/yolov5", "yolov5s")

# Images
for f in "zidane.jpg", "bus.jpg":
    torch.hub.download_url_to_file("https://ultralytics.com/images/" + f, f)  # download 2 images
im1 = Image.open("zidane.jpg")  # PIL image
im2 = cv2.imread("bus.jpg")[..., ::-1]  # OpenCV image (BGR to RGB)
im3 = cv2.imread("test1.jpg")[..., ::-1] 

print(im2.min())
print(im2.max())

# Inference
results = model([im1, im2, im3], size=640)  # batch of images

# Results
results.print()
results.save()  # or .show()

print(results.xyxy[0])  # im1 predictions (tensor)
print(results.pandas().xyxy[0]) 

