import cv2, time
from picamera2 import Picamera2, Preview
from libcamera import ColorSpace
from PIL import Image
import numpy as np

def take_pic():
    picam2 = Picamera2()
    cv2.startWindowThread()
    camera_config = picam2.create_still_configuration(main={"format": 'BGR888', "size": (3280, 2464)})
    picam2.configure(camera_config)
    #picam2.start_preview(Preview.NULL)
    picam2.start()


    time.sleep(1)
    image = picam2.capture_image("main")
    reg_image = np.array(image)

    print(reg_image.shape)

    picam2.switch_mode_and_capture_file(camera_config,"camera/src/Image/image.jpg")

    

im = Image.open(r"/home/g751/Desktop/Project/P7-751/camera/src/Image/image.jpg")

im_array = np.array(im)

print(im_array.shape)

lower_red = np.array([150,0,0], dtype="uint8")
upper_red = np.array([255,75,75], dtype="uint8")
lower_blue = np.array([0,0,150], dtype="uint8")
upper_blue = np.array([75,75,255], dtype="uint8")
lower_green = np.array([0,100,0], dtype="uint8")
upper_green = np.array([100,255,100], dtype="uint8")
lower_yellow = np.array([150,150,0], dtype="uint8")
upper_yellow = np.array([255,255,75], dtype="uint8")


mask_red = cv2.inRange(im_array,lower_red,upper_red)
mask_blue = cv2.inRange(im_array,lower_blue, upper_blue)
mask_green = cv2.inRange(im_array,lower_green, upper_green)
mask_yellow = cv2.inRange(im_array,lower_yellow, upper_yellow)

masked_red = cv2.bitwise_and(im_array,im_array,mask=mask_red)
masked_blue = cv2.bitwise_and(im_array,im_array,mask=mask_blue)
masked_green = cv2.bitwise_and(im_array,im_array,mask=mask_green)
masked_yellow = cv2.bitwise_and(im_array,im_array,mask=mask_yellow)

res_image = masked_red + masked_blue + masked_yellow + masked_green
cv2.imshow("masked",res_image)
cv2.waitKey(0)
#cv2.imshow("red",masked_red)
#cv2.waitKey(0)
#cv2.imshow("blue",masked_blue)
#cv2.waitKey(0)
cv2.imshow("green",masked_green)
cv2.waitKey(0)
#cv2.imshow("yellow",masked_yellow)
#cv2.waitKey(0)













