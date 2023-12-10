import cv2, time
#from picamera2 import Picamera2, Preview
#from libcamera import ColorSpace
from PIL import Image
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import colors

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

    picam2.switch_mode_and_capture_file(camera_config,"camera/src/Image/blueimage.jpg")

image_center = [int(2464/2),int(3280/2)]    

#im = cv2.imread("/home/g751/Desktop/Project/P7-751/camera/src/Image/newimage.jpg")
#im = cv2.imread("/home/christian/Drone_project/P7-751/camera/src/Image/newimage.jpg")

#im_rgb = cv2.cvtColor(im,cv2.COLOR_RGB2BGR)

#im2 = Image.open("/home/g751/Desktop/Project/P7-751/camera/src/Image/newimage.jpg")
#im_blue = cv2.imread("/home/g751/Desktop/Project/P7-751/camera/src/Image/blueimage.jpg")

im = Image.open("/home/christian/Drone_project/P7-751/camera/src/Image/newimage.jpg")
im_blue = Image.open("/home/christian/Drone_project/P7-751/camera/src/Image/blueimage.jpg")
im_red = Image.open("/home/christian/Drone_project/P7-751/camera/src/Image/RED.png")
im_green = Image.open("/home/christian/Drone_project/P7-751/camera/src/Image/GREEN.png")


im_array = np.array(im)
im_blue_array = np.array(im_blue)
im_red_array = np.array(im_red)
im_green_array = np.array(im_green)





# MIN_MATCH_COUNT = 1

# sift = cv2.SIFT_create()

# kp1, des1 = sift.detectAndCompute(im_array, None)
# kp2, des2 = sift.detectAndCompute(im_blue_array, None)

# FLANN_INDEX_KDTREE = 1
# index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
# search_params = dict(checks = 100)

# flann = cv2.FlannBasedMatcher(index_params, search_params)

# matches = flann.knnMatch(des1, des2, k=2)

# good = []

# for m,n in matches:
#     if m.distance < 0.7*n.distance:
#         good.append(m)

# if len(good)> MIN_MATCH_COUNT:
#     src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1,1,2)
#     dst_pts = np.float32([kp1[m.trainIdx].pt for m in good]).reshape(-1,1,2)

#     M, mask = cv2.findHomography(src_pts, cv2.RANSAC, 5)
#     matchesMask = mask.ravel().tolist()

#     h,w = im_array.shape
#     pts = np.float32([[0,0],[0,h-1],[w-1,h-1],[w-1,0]]).reshape(-1,1,2)
#     dst = cv2.perspectiveTransform(pts,M)

#     im_blue_array = cv2.polylines(im_blue_array,[np.int32(dst)],True,255,3,cv2.LINE_AA)

# else:
#    print("Not enough matches found - {}/{}".format(len(good),MIN_MATCH_COUNT))
# matchesMask = None

# draw_params = dict(matchColor = (0,255,0),
#     singlePointColor = None,
#     matchesMask = matchesMask,
#     flags = 2)

# img3 = cv2.drawMatches(im_array,kp1,im_blue_array,kp2, good,None,**draw_params)

# plt.imshow(img3,'gray'),plt.show

# w, h = im_blue.shape[:-1]


# threshold = 0.8

# im_array_R, im_array_G, im_array_B = cv2.split(im)
# im_blue_array_R, im_blue_array_G, im_blue_array_B = cv2.split(im_blue)

# resultB = cv2.matchTemplate(im_array_R,im_blue_array_R, cv2.TM_SQDIFF)
# resultG = cv2.matchTemplate(im_array_G,im_blue_array_G, cv2.TM_SQDIFF)
# resultR = cv2.matchTemplate(im_array_B,im_blue_array_B, cv2.TM_SQDIFF)


# result = resultB + resultG + resultR



#print("result : ", resultR)
#print(result)
#loc = np.where(result >= 3*threshold)
#print("loc:",loc)
#print(loc.index(0))

# min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(loc)

# top_left = min_loc
# bottom_right = (top_left[0] + w, top_left[1] + h)

# cv2.rectangle(im2_array, top_left, bottom_right, 255 , 2)

# plt.imshow(im2_array)
# plt.title("detected point"), plt.xticks([]),plt.yticks([])
# plt.show()

#min_idk_x = np.min(loc.index(0)[:])
#max_idk_x = np.max(loc.index(0)[:])
#min_idk_y = np.min(loc.index(1)[:])
#max_idk_y = np.max(loc.index(1)[:])

#print(min_idk_x, max_idk_x, min_idk_y, max_idk_y)


# orb = cv2.ORB_create()

# im_bgr = cv2.cvtColor(im2_array,cv2.COLOR_RGB2BGR)
# im_blue_bgr = cv2.cvtColor(im_blue_array, cv2.COLOR_RGB2BGR)


# kp, des = orb.detectAndCompute(im_bgr,None)
# kp_blue, des_blue = orb.detectAndCompute(im_blue_bgr,None)

# bf = cv2.BFMatcher(cv2.NORM_HAMMING2, crossCheck= False)

# matches_blue = bf.match(des,des_blue)

# print(matches_blue)

# matches_blue = sorted(matches_blue, key = lambda x:x.distance)

# print(matches_blue)

# img3 = cv2.drawMatches(im2_array,kp,im_blue_array,kp_blue,matches_blue[:10],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

# plt.imshow(img3),plt.show()



# img_blue=(im_blue_array.astype(float))/255.0
# im_blue_hsv = colors.rgb_to_hsv(img_blue[...,:3])
# img_hsv_blue=im_blue_hsv[...,0].flatten()
                
# plt.hist(img_hsv_blue*360,256,range=(0.0,256), label="Hue")
# plt.show()

# img_red=(im_red.astype(float))/255.0
# im_red_hsv = colors.rgb_to_hsv(img_red[...,:3])
# img_hsv_red=im_red_hsv[...,0].flatten()
                
# plt.hist(img_hsv_red*360,256,range=(0.0,256), label="Hue")
# plt.show()


# def create_prob_distr(img, histogram, min_saturation, min_value, min_prob):    
#     (h_values,bin_edges,n) = histogram
#     h_values = min_max_scaling(h_values)
#     img = (img.astype(float))/255.0
#     img_hsv = colors.rgb_to_hsv(img[...,:3])
#     prob_distr = np.zeros((img_hsv.shape[0], img_hsv.shape[1]))    
#     for i in range(img_hsv.shape[0]):
#         for j in range(img_hsv.shape[1]):
#             bin_index = np.digitize(img_hsv[i][j][0]*360,bin_edges, right=True)            
#             if(img_hsv[i][j][1] < min_saturation or img_hsv[i][j][2] < min_value or h_values[bin_index-1] < min_prob):                
#                 prob_distr[i][j] = 0.0            
#             else:                
#                 prob_distr[i][j] = h_values[bin_index-1]
#         return prob_distr

#im_green = np.array(cv2.cvtColor(im_green_array,cv2.COLOR_BGR2HSV))


# im = (im_array.astype(float))/255.0
# im = colors.rgb_to_hsv(im_array[...,:3])

# im_green = (im_green_array.astype(float))/255.0
# im_green = colors.rgb_to_hsv(im_green_array[...,:3])

# print("im green HSV", im)

# lower_green_hsv = np.array([110/255,30/255,20/255])
# upper_green_hsv = np.array([150/255,70/255,50/255])

# print("Lower HSV green: ", lower_green_hsv)
# print("upper green hsv:",upper_green_hsv)


# Create Blob detector parameters

# params = cv2.SimpleBlobDetector_Params()


# params.filterByArea = True
# params.minArea = 100

# Set circularity parameter to true and set min to close to perfect circle

# params.filterByCircularity = False
# params.minCircularity = 0.6
# params.maxCircularity = 1

# Set convexity
# params.filterByConvexity = True
# params.minConvexity = 0.87

# params.filterByInertia = False

# Creates the detector with the given parameters
# ver = (cv2.__version__).split('.')
# if int(ver[0]) < 3 :
#     detector = cv2.SimpleBlobDetector(params)
# else : 
#     detector = cv2.SimpleBlobDetector_create(params)


# Turn image into HSV colorspace
im_array_hsv = cv2.cvtColor(im_array, cv2.COLOR_RGB2HSV)



# Create color boundaries in HSV spectrum 
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

# in RGB
# lower_red = np.array([150,0,0], dtype="uint8")
# upper_red = np.array([255,75,75], dtype="uint8")
# lower_blue = np.array([0,0,100], dtype="uint8")
# upper_blue = np.array([100,100,255], dtype="uint8")
# lower_green = np.array([0,75,0], dtype="uint8")
# upper_green = np.array([75,255,75], dtype="uint8")
# lower_yellow = np.array([150,150,0], dtype="uint8")
# upper_yellow = np.array([255,255,75], dtype="uint8")

# Create the different masks using the boundaries set before
mask_red_end = cv2.inRange(im_array_hsv,lower_red_end,upper_red_end)
mask_red = cv2.inRange(im_array_hsv,lower_red,upper_red)
mask_blue = cv2.inRange(im_array_hsv,lower_blue, upper_blue)
mask_green = cv2.inRange(im_array_hsv,lower_green, upper_green)
mask_yellow = cv2.inRange(im_array_hsv,lower_yellow, upper_yellow)

# Add the two masks for red as their are in opposite ends of the HSV spectrum
mask_red = cv2.bitwise_or(mask_red,mask_red_end)


# print("Mask Green shape:", mask_green.shape)
# print("mask REd:", mask_red)

# Perform bitwise and operation on the image with the set masks to set all pixels that arent that color
# to 0
masked_red = cv2.bitwise_and(im_array,im_array,mask=mask_red)
masked_blue = cv2.bitwise_and(im_array,im_array,mask=mask_blue)
masked_green = cv2.bitwise_and(im_array,im_array,mask=mask_green)
masked_yellow = cv2.bitwise_and(im_array,im_array,mask=mask_yellow)

# print("Masked Green:",masked_green)
# print("masked Red:", masked_red)

# Median blur the image to remove salt and pepper noise and then perform opening to eliminate more noise
masked_red_median = cv2.medianBlur(masked_red,3)
masked_red_median_opening = cv2.morphologyEx(masked_red_median, cv2.MORPH_OPEN, np.ones((7,7),np.uint8))
masked_blue_median = cv2.medianBlur(masked_blue,3)
masked_blue_median_opening = cv2.morphologyEx(masked_blue_median, cv2.MORPH_OPEN, np.ones((7,7),np.uint8))
masked_green_median = cv2.medianBlur(masked_green,3)
masked_green_median_opening = cv2.morphologyEx(masked_green_median, cv2.MORPH_OPEN, np.ones((7,7),np.uint8))
masked_yellow_median = cv2.medianBlur(masked_yellow,3)
masked_yellow_median_opening = cv2.morphologyEx(masked_yellow_median, cv2.MORPH_OPEN, np.ones((7,7),np.uint8))

# transpose the array and remove zero elements
mask_red_filtered = np.transpose(np.nonzero(masked_red_median_opening[:,:,0] > 0)) 
mask_blue_filtered = np.transpose(np.nonzero(masked_blue_median_opening[:,:,0] > 0)) 
mask_green_filtered = np.transpose(np.nonzero(masked_green_median_opening[:,:,0] > 0)) 
mask_yellow_filtered = np.transpose(np.nonzero(masked_yellow_median_opening[:,:,0] > 0)) 

# print("mask green filtered:", mask_green_filtered)

# print("Max x:" + str(np.max(mask_red_filtered[:, 0])))
# print("Min x:" + str(np.min(mask_red_filtered[:, 0])))
# print("Max y:" + str(np.max(mask_red_filtered[:, 1])))
# print("Min y:" + str(np.min(mask_red_filtered[:, 1])))
# print("Max x:" + str(np.max(mask_blue_filtered[:, 0])))
# print("Min x:" + str(np.min(mask_blue_filtered[:, 0])))
# print("Max y:" + str(np.max(mask_blue_filtered[:, 1])))
# print("Min y:" + str(np.min(mask_blue_filtered[:, 1])))
# print("Max x:" + str(np.max(mask_green_filtered[:, 0])))
# print("Min x:" + str(np.min(mask_green_filtered[:, 0])))

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

# Calculate the center of each color marker
red_center = [int((red_x_max - red_x_min)/2 + red_x_min), int((red_y_max - red_y_min)/2 + red_y_min)]
blue_center = [int((blue_x_max - blue_x_min)/2 + blue_x_min), int((blue_y_max - blue_y_min)/2 + blue_y_min)]
green_center = [int((green_x_max - green_x_min)/2 + green_x_min), int((green_y_max - green_y_min)/2 + green_y_min)] 
yellow_center = [int((yellow_x_max - yellow_x_min)/2 + yellow_x_min), int((yellow_y_max - yellow_y_min)/2 + yellow_y_min)]

# print(red_center)
# print(blue_center)
# print(green_center)
# print(yellow_center)
# print(masked_red.shape)
# print(green_x_max,green_x_min)

# Add all the mased images together
res_image = masked_red_median_opening + masked_blue_median_opening + masked_yellow_median_opening + masked_green_median_opening

# Convert to grayscale
# masked_red_gray = cv2.cvtColor(masked_red_median_opening,cv2.COLOR_RGB2GRAY)
# masked_blue_gray = cv2.cvtColor(masked_blue_median_opening,cv2.COLOR_RGB2GRAY)
# masked_green_gray = cv2.cvtColor(masked_green_median_opening,cv2.COLOR_RGB2GRAY)
# masked_yellow_gray = cv2.cvtColor(masked_yellow_median_opening,cv2.COLOR_RGB2GRAY)

# masked_blue_gray = cv2.resize(masked_blue_gray,[960,540])
# cv2.imshow("gray",masked_blue_gray)
# cv2.waitKey(0)

# Threshold
# _, masked_red_gray = cv2.threshold(masked_red_gray, 127,255,cv2.THRESH_BINARY)
# _, masked_blue_gray = cv2.threshold(masked_blue_gray, 50,255,cv2.THRESH_BINARY)

#masked_blue_gray = cv2.bitwise_and(masked_blue_gray,masked_blue_gray)
# _,masked_green_gray = cv2.threshold(masked_green_gray, 127,255,cv2.THRESH_BINARY)
# _,masked_yellow_gray = cv2.threshold(masked_yellow_gray, 127,255,cv2.THRESH_BINARY)

# masked_blue_gray = cv2.resize(masked_blue_gray,[960,540])
# cv2.imshow("thresh", masked_blue_gray)
# cv2.waitKey(0)



# Perform blob detection
# keypoints_red = detector.detect(masked_red_gray)
# keypoints_blue = detector.detect(masked_blue_gray)
# keypoints_green = detector.detect(masked_green_gray)
# keypoints_yellow = detector.detect(masked_yellow_gray)



# im_keypoints_red = cv2.drawKeypoints(masked_red_gray, keypoints_red, None, (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
# im_keypoints_blue = cv2.drawKeypoints(masked_blue_gray, keypoints_blue, None, (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
# im_keypoints_green = cv2.drawKeypoints(masked_green_gray, keypoints_green, None, (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
# im_keypoints_yellow = cv2.drawKeypoints(masked_yellow_gray, keypoints_yellow, None, (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

#im_keypoints = cv2.cvtColor(im_keypoints, cv2.COLOR_RGB2HSV)

# im_keypoints = cv2.resize(im_keypoints_red,[960,540])
# cv2.imshow("blobs",im_keypoints)
# cv2.waitKey(0)

# Draw circles at the center of each color marker for visualizaion
res_image_circle_red = cv2.circle(masked_red_median_opening,(red_center[1],red_center[0]),10,(255,255,255),-30)
res_image_circle_blue = cv2.circle(masked_blue_median_opening,(blue_center[1],blue_center[0]),10,(255,255,255),-30)
res_image_circle_green = cv2.circle(masked_green_median_opening,(green_center[1],green_center[0]),10,(255,255,255),-30)
res_image_circle_yellow = cv2.circle(masked_yellow_median_opening,(yellow_center[1],yellow_center[0]),10,(255,255,255),-30)

# Calculate the center of the 4 colored markers
center_point = [int((red_center[1] + blue_center[1] + green_center[1] + yellow_center[1])/4), int((red_center[0] + blue_center[0] + green_center[0] + yellow_center[0])/4)]

# # Draw a circle in the center of the of 4 colored markers
res_image_circle_center = cv2.circle(masked_yellow_median_opening,(center_point[0],center_point[1]),30,(255,255,255),-60)

# Draw circle in center of image
res_image_center = cv2.circle(masked_yellow_median_opening,(image_center[1],image_center[0]),30,(0,0,255),-60)

# Add the different images together
res_image_circle = res_image_circle_red + res_image_circle_blue + res_image_circle_green + res_image_circle_yellow + res_image_circle_center + res_image_center

# Rezise because opencv is lazy
res_image_circle = cv2.resize(res_image_circle, [960,540])


#median = cv2.medianBlur(res_image, 5)

cv2.imshow("circle", res_image_circle)
cv2.waitKey(0)
#cv2.imshow("green_dot",res_image_circle_green)
#cv2.waitKey(0)
#cv2.imshow("masked",res_image)
#cv2.waitKey(0)
#cv2.imshow("MedianBlured",median)
#cv2.waitKey(0)
# cv2.imshow("red",masked_red)
# cv2.waitKey(0)
#cv2.imshow("blue",masked_blue)
#cv2.waitKey(0)
#cv2.imshow("green",masked_green)
#cv2.waitKey(0)
#cv2.imshow("yellow",masked_yellow)
#cv2.waitKey(0)












