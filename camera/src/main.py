import cv2, time
from picamera2 import Picamera2, Preview
from libcamera import ColorSpace
from PIL import Image
import numpy as np


picam2 = Picamera2()
cv2.startWindowThread()
camera_config = picam2.create_preview_configuration(main={"format": 'BGR888', "size": (640, 480)})
picam2.configure(camera_config)
picam2.start_preview(Preview.NULL)
picam2.start()


time.sleep(1)
image = picam2.capture_image("main")
reg_image = np.array(image)
cv2.imwrite("/Desktop/something.jpg", reg_image)




