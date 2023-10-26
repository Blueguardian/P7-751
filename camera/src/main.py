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
lower_green = np.array([0,75,0], dtype="uint8")
upper_green = np.array([75,255,75], dtype="uint8")
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


masked_red_median = cv2.medianBlur(masked_red,3)
masked_red_median_opening = cv2.morphologyEx(masked_red_median, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
masked_blue_median = cv2.medianBlur(masked_blue,3)
masked_blue_median_opening = cv2.morphologyEx(masked_blue_median, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
masked_green_median = cv2.medianBlur(masked_green,3)
masked_green_median_opening = cv2.morphologyEx(masked_green_median, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
masked_yellow_median = cv2.medianBlur(masked_yellow,3)
masked_yellow_median_opening = cv2.morphologyEx(masked_yellow_median, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))


mask_red_filtered = np.transpose(np.nonzero(masked_red_median_opening[:,:,0] > 0)) 
mask_blue_filtered = np.transpose(np.nonzero(masked_blue_median_opening[:,:,0] > 0)) 
mask_green_filtered = np.transpose(np.nonzero(masked_green_median_opening[:,:,0] > 0)) 
mask_yellow_filtered = np.transpose(np.nonzero(masked_yellow_median_opening[:,:,0] > 0)) 
#print(mask_red_filtered.shape)

print("Max x:" + str(np.max(mask_red_filtered[:, 0])))
print("Min x:" + str(np.min(mask_red_filtered[:, 0])))
print("Max y:" + str(np.max(mask_red_filtered[:, 1])))
print("Min y:" + str(np.min(mask_red_filtered[:, 1])))
print("Max x:" + str(np.max(mask_blue_filtered[:, 0])))
print("Min x:" + str(np.min(mask_blue_filtered[:, 0])))
print("Max y:" + str(np.max(mask_blue_filtered[:, 1])))
print("Min y:" + str(np.min(mask_blue_filtered[:, 1])))

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


red_center = [int((red_x_max - red_x_min)/2 + red_x_min), int((red_y_max - red_y_min)/2 + red_y_min)]
blue_center = [int((blue_x_max - blue_x_min)/2 + blue_x_min), int((blue_y_max - blue_y_min)/2 + blue_y_min)]
green_center = [int((green_x_max - green_x_min)/2 + green_x_min), int((green_y_max - green_y_min)/2 + green_y_min)] 
yellow_center = [int((yellow_x_max - yellow_x_min)/2 + yellow_x_min), int((yellow_y_max - yellow_y_min)/2 + yellow_y_min)]

print(red_center)
print(blue_center)
print(green_center)
print(yellow_center)
print(masked_red.shape)
print(green_x_max,green_x_min)


res_image = masked_red + masked_blue + masked_yellow + masked_green

res_image_circle_red = cv2.circle(masked_red_median_opening,(red_center[1],red_center[0]),30,(255,255,255),-60)
res_image_circle_blue = cv2.circle(masked_blue_median_opening,(blue_center[1],blue_center[0]),30,(255,255,255),-60)
res_image_circle_green = cv2.circle(masked_green_median_opening,(green_center[1],green_center[0]),30,(255,255,255),-60)
res_image_circle_yellow = cv2.circle(masked_yellow_median_opening,(yellow_center[1],yellow_center[0]),30,(255,255,255),-60)

res_image_circle = res_image_circle_red + res_image_circle_blue + res_image_circle_green + res_image_circle_yellow

#median = cv2.medianBlur(res_image, 5)

cv2.imshow("circle", res_image_circle)
cv2.waitKey(0)
#cv2.imshow("green_dot",res_image_circle_green)
#cv2.waitKey(0)
#cv2.imshow("masked",res_image)
#cv2.waitKey(0)
#cv2.imshow("MedianBlured",median)
#cv2.waitKey(0)
#cv2.imshow("red",masked_red)
#cv2.waitKey(0)
#cv2.imshow("blue",masked_blue)
#cv2.waitKey(0)
#cv2.imshow("green",masked_green)
#cv2.waitKey(0)
#cv2.imshow("yellow",masked_yellow)
#cv2.waitKey(0)













