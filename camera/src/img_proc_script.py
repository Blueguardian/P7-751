import cv2, time
from picamera2 import Picamera2, Preview
from libcamera import ColorSpace
from PIL import Image
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import colors
import os

# Global variables declaration
query_pic = 0
query_img = 0
image_center = [int(2464/2),int(3280/2)]

script_dir = os.path.dirname(__file__)
rel_path = "Image"
abs_file_path = os.path.join(script_dir, rel_path)

lower_red = np.array([0,100,100])
upper_red = np.array([10,255,255])
lower_red_end = np.array([170,100,100])
upper_red_end = np.array([179,255,255])

lower_blue = np.array([110,100,100])
upper_blue = np.array([130,255,255])

lower_green = np.array([40,50,50])
upper_green = np.array([80,255,255])

lower_yellow = np.array([20,100,100])
upper_yellow = np.array([35,255,255])

picam2 = Picamera2()
cv2.startWindowThread()
camera_config = picam2.create_still_configuration(main={"format": 'BGR888', "size": (3280, 2464)})
picam2.configure(camera_config)
#picam2.start_preview(Preview.NULL)
picam2.start()

def take_pic():
    global query_pic
    print("Taking picture...")

    time.sleep(1)
    image = picam2.capture_image("main")
    reg_image = np.array(image)

    print(reg_image.shape)
    if query_pic == 0:
        picam2.switch_mode_and_capture_file(camera_config,"camera/src/Image/Referenceimage.jpg")
        query_pic += 1
        print("Took reference picture...")
    if query_pic == 1:
        picam2.switch_mode_and_capture_file(camera_config,"camera/src/Image/Queryimage.jpg")
        print("Took query picture...")
    print("Done taking picture!")

def imageProc(image):

    print("starting image magic...")
    # Convert to HSV
    image_hsv = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)

    # Create the different masks using the boundaries set before
    mask_red_end = cv2.inRange(image_hsv,lower_red_end,upper_red_end)
    mask_red = cv2.inRange(image_hsv,lower_red,upper_red)
    mask_blue = cv2.inRange(image_hsv,lower_blue, upper_blue)
    mask_green = cv2.inRange(image_hsv,lower_green, upper_green)
    mask_yellow = cv2.inRange(image_hsv,lower_yellow, upper_yellow)

    # Add the two masks for red as their are in opposite ends of the HSV spectrum
    mask_red = cv2.bitwise_or(mask_red,mask_red_end)

    # Perform bitwise and operation on the image with the set masks to set all pixels that arent that color
    # to 0
    masked_red = cv2.bitwise_and(image,image,mask=mask_red)
    masked_blue = cv2.bitwise_and(image,image,mask=mask_blue)
    masked_green = cv2.bitwise_and(image,image,mask=mask_green)
    masked_yellow = cv2.bitwise_and(image,image,mask=mask_yellow)

    # Median blur the image to remove salt and pepper noise and then perform opening to eliminate more noise
    masked_red_median = cv2.medianBlur(masked_red,3)
    masked_red_median_opening = cv2.morphologyEx(masked_red_median, cv2.MORPH_OPEN, np.ones((7,7),np.uint8))
    masked_blue_median = cv2.medianBlur(masked_blue,3)
    masked_blue_median_opening = cv2.morphologyEx(masked_blue_median, cv2.MORPH_OPEN, np.ones((7,7),np.uint8))
    masked_green_median = cv2.medianBlur(masked_green,3)
    masked_green_median_opening = cv2.morphologyEx(masked_green_median, cv2.MORPH_OPEN, np.ones((7,7),np.uint8))
    masked_yellow_median = cv2.medianBlur(masked_yellow,3)
    masked_yellow_median_opening = cv2.morphologyEx(masked_yellow_median, cv2.MORPH_OPEN, np.ones((7,7),np.uint8))

    print("Abracadabra it its now filtered...")

    # transpose the array and remove zero elements
    mask_red_filtered = np.transpose(np.nonzero(masked_red_median_opening[:,:,0] > 0)) 
    mask_blue_filtered = np.transpose(np.nonzero(masked_blue_median_opening[:,:,0] > 0)) 
    mask_green_filtered = np.transpose(np.nonzero(masked_green_median_opening[:,:,0] > 0)) 
    mask_yellow_filtered = np.transpose(np.nonzero(masked_yellow_median_opening[:,:,0] > 0)) 

    # Find maximum and minimum values for x and y in all 4 color markers
    red_x_max = np.max(mask_red_filtered[:, 0])
    red_x_min =np.min(mask_red_filtered[:, 0])
    red_y_max =np.max(mask_red_filtered[:, 1])
    red_y_min =np.min(mask_red_filtered[:, 1])
    blue_x_max =np.max(mask_blue_filtered[:, 0])
    blue_x_min =np.min(mask_blue_filtered[:, 0])
    blue_y_max =np.max(mask_blue_filtered[:, 1])
    blue_y_min =np.min(mask_blue_filtered[:, 1])
    green_x_max =np.max(mask_green_filtered[:, 0])
    green_x_min =np.min(mask_green_filtered[:, 0])
    green_y_max =np.max(mask_green_filtered[:, 1])
    green_y_min =np.min(mask_green_filtered[:, 1])
    yellow_x_max =np.max(mask_yellow_filtered[:, 0])
    yellow_x_min =np.min(mask_yellow_filtered[:, 0])
    yellow_y_max =np.max(mask_yellow_filtered[:, 1])
    yellow_y_min =np.min(mask_yellow_filtered[:, 1])

    print("Calculated middle points...")
    # Calculate the center of each color marker
    red_center = [int((red_x_max - red_x_min)/2 + red_x_min), int((red_y_max - red_y_min)/2 + red_y_min)]
    blue_center = [int((blue_x_max - blue_x_min)/2 + blue_x_min), int((blue_y_max - blue_y_min)/2 + blue_y_min)]
    green_center = [int((green_x_max - green_x_min)/2 + green_x_min), int((green_y_max - green_y_min)/2 + green_y_min)] 
    yellow_center = [int((yellow_x_max - yellow_x_min)/2 + yellow_x_min), int((yellow_y_max - yellow_y_min)/2 + yellow_y_min)]

    # Calculate the center of the 4 colored markers
    center_point = [int((red_center[1] + blue_center[1] + green_center[1] + yellow_center[1])/4), int((red_center[0] + blue_center[0] + green_center[0] + yellow_center[0])/4)]

    print("Calculated center point and finished magic!")
    
    return center_point


def calculateDifference(center_point_reference,center_point_query):
    print("Calculating difference in center point...")
    calc_diff_x = center_point_reference[1] - center_point_query[1]
    calc_diff_y = center_point_reference[0] - center_point_query[0]

    euc_dist = np.sqrt(((image_center[1]- center_point_query[1])*(image_center[1]- center_point_query[1]))+((image_center[0]-center_point_query[0])*(image_center[0]-center_point_query[0])))
    calculated_difference = [calc_diff_x,calc_diff_y]
    print("Done calculating difference! It is:", calculated_difference)
    print("Euclidean Distance:",euc_dist)
    return calculated_difference

while True:

    take_pic()

    im_reference = np.array(Image.open(str(abs_file_path)+"/Referenceimage.jpg"))
    im_query = np.array(Image.open(str(abs_file_path)+"/Queryimage.jpg"))

    if query_img == 0:
        center_point_reference = imageProc(im_reference)
        query_img += 1
    if query_img == 1:
        center_point_query = imageProc(im_query)
    
    # cv2.imshow("query", im_query)
    # cv2.waitKey(0)
    calculateDifference(center_point_reference,center_point_query)
    





