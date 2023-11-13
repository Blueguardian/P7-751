import cv2, time
from picamera2 import Picamera2, Preview
from libcamera import ColorSpace
from PIL import Image
import numpy as np
import os

script_dir = os.path.dirname(__file__)
rel_path = "Images"
abs_file_path = os.path.join(script_dir, rel_path)

i = 0

picam2 = Picamera2()
cv2.startWindowThread()
camera_config = picam2.create_still_configuration(main={"format": 'BGR888', "size": (3280, 2464)})
picam2.configure(camera_config)
#picam2.start_preview(Preview.NULL)
picam2.start()

time.sleep(1)
image = picam2.capture_image("main")
reg_image = np.array(image)

while i < 20:
    picam2.switch_mode_and_capture_file(camera_config,"camera/src/Images/Chess.jpg"+str(i))
    cv2.imshow("chess:"+str(i), str(abs_file_path)+"Chess"+str(i))
    cv2.waitKey(0)
    i += 1
    



