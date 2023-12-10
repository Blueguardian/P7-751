import cv2, time
from picamera2 import Picamera2
from PIL import Image
import numpy as np
import os
import multiprocessing
from multiprocessing import SimpleQueue, Lock
import sys

from Communication.xmlhandler import XMLhandler
from Communication.tcp_server import TCPServer
from Communication.Python_serial_com import Teensy_comm

# Global variables declaration
query_pic = 0
query_img = 0
image_center = [int(2464/2),int(3280/2)]

script_dir = os.path.dirname(__file__)
rel_path = "Image"
abs_file_path = os.path.join(script_dir, rel_path)

lower_red = np.array([0,150,150])
upper_red = np.array([10,255,255])
lower_red_end = np.array([170,150,150])
upper_red_end = np.array([179,255,255])

lower_blue = np.array([105,100,100])
upper_blue = np.array([130,255,255])

lower_green = np.array([40,50,50])
upper_green = np.array([80,255,255])

lower_yellow = np.array([20,100,100])
upper_yellow = np.array([35,255,255])

#focal length
f_x = 2770.28
f_y = 2765.57
u1 = 1602.71
v1 = 1200.56

k_instrinsic = np.array([[f_x,0,u1],
                         [0,f_y,v1],
                         [0,0,1.0]],dtype="double")

k_instrinsic_inverted = np.linalg.inv(k_instrinsic)

dist_coef = np.array([0.175, 0.133, -0.002, -0.008, -1.965],dtype="double")

# Picam setup



# Hard coded global coordinates for markers in mm [x,y,z]

red = [20,20,0]
blue = [8.7,20,0]
green = [20,7,0]
yellow = [8.7,7,0]

object_points = np.array([red,
                 blue,
                 green,
                 yellow], dtype="double")


def takePic(picam2, camera_config):
    print("im in takepic")
    
    #cv2.startWindowThread()
    #picam2.start_preview(Preview.NULL)
    
    global query_pic
    print("Taking picture...")

    #time.sleep(0.2)
    #image = picam2.capture_image("main")
    #reg_image = np.array(image)

    #print(reg_image.shape)
    
    # if query_pic == 0:
    #     print("query pic = 0")
    #     picam2.switch_mode_and_capture_file(camera_config,"camera/src/Image/Referenceimage.jpg")
    #     query_pic += 1
    #     print("Took reference picture...")
    #if query_pic == 1:
    print("query pic = 1")
    query = picam2.capture_array('main')
    query = cv2.resize(query,[1080,1920])
    print("Took query picture...")
    print("Done taking picture!")
    return query

# def phantomMarker(red,blue,green,yellow):
#
#     #code here
#     i = 1

    

def imageProc(image):

    dummy_return = 0

    global lower_red     
    global upper_red 
    global lower_red_end 
    global upper_red_end 
    global lower_blue  
    global upper_blue 
    global lower_green    
    global upper_green  
    global lower_yellow 
    global upper_yellow  

    global image_center
    #print(image)

    
    print("starting image magic...")
    # Convert to HSV
    image_hsv = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)

    #print("image shape:",image_hsv.shape)
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
    #print(masked_red)

    # Median blur the image to remove salt and pepper noise and then perform opening to eliminate more noise
    masked_red_median = cv2.medianBlur(masked_red,3)
    masked_red_median_opening = cv2.morphologyEx(masked_red_median, cv2.MORPH_OPEN, np.ones((7,7),np.uint8))
    masked_blue_median = cv2.medianBlur(masked_blue,3)
    masked_blue_median_opening = cv2.morphologyEx(masked_blue_median, cv2.MORPH_OPEN, np.ones((7,7),np.uint8))
    masked_green_median = cv2.medianBlur(masked_green,3)
    masked_green_median_opening = cv2.morphologyEx(masked_green_median, cv2.MORPH_OPEN, np.ones((7,7),np.uint8))
    masked_yellow_median = cv2.medianBlur(masked_yellow,3)
    masked_yellow_median_opening = cv2.morphologyEx(masked_yellow_median, cv2.MORPH_OPEN, np.ones((7,7),np.uint8))

    # cv2.imshow("blue",masked_blue_median_opening)
    # cv2.waitKey(0)
    print("Abracadabra it its now filtered...")

    # transpose the array and remove zero elements
    mask_red_filtered = np.transpose(np.nonzero(masked_red_median_opening[:,:,0] > 0)) 
    mask_blue_filtered = np.transpose(np.nonzero(masked_blue_median_opening[:,:,0] > 0)) 
    mask_green_filtered = np.transpose(np.nonzero(masked_green_median_opening[:,:,0] > 0)) 
    mask_yellow_filtered = np.transpose(np.nonzero(masked_yellow_median_opening[:,:,0] > 0)) 


    if mask_red_filtered.size == 0:
        
        print("Mask red was 0, exiting function...")
        return dummy_return
    if mask_blue_filtered.size == 0:
        
        print("Mask blue was 0, exiting function...")
        return dummy_return
    if mask_green_filtered.size == 0:
        
        print("Mask green was 0, exiting function...")
        return dummy_return
    if mask_yellow_filtered.size == 0:
        
        print("Mask yellow was 0, exiting function...")
        return dummy_return



#    print("red:", mask_red_filtered.shape)
    #print("blue:", mask_blue_filtered.shape)
    #print("green:", mask_green_filtered.shape)
    #print("yellow:", mask_yellow_filtered.shape)
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
    #center_point = [int((red_center[1] + blue_center[1] + green_center[1] + yellow_center[1])/4), int((red_center[0] + blue_center[0] + green_center[0] + yellow_center[0])/4)]

    center_coords = np.array([red_center[0],red_center[1],blue_center[0],blue_center[1],green_center[0],green_center[1],yellow_center[0],yellow_center[1]])

    print("Calculated center point and finished magic!")
    
    return center_coords


def calculateDifference(center_point_reference,center_point_query):
    print("Calculating difference in center point...")
    calc_diff_x = center_point_reference[1] - center_point_query[1]
    calc_diff_y = center_point_reference[0] - center_point_query[0]

    euc_dist = np.sqrt(((image_center[1]- center_point_query[1])*(image_center[1]- center_point_query[1]))+((image_center[0]-center_point_query[0])*(image_center[0]-center_point_query[0])))
    calculated_difference = [calc_diff_x,calc_diff_y]
    print("Done calculating difference! It is:", calculated_difference)
    print("Euclidean Distance:",euc_dist)
    return calculated_difference

def calculateRotationTranslation(center_coords_ref,center_coords_query):

    red_marker = [center_coords_query[1],center_coords_query[0]]
    blue_marker = [center_coords_query[3],center_coords_query[2]]
    green_marker = [center_coords_query[5],center_coords_query[4]]
    yellow_marker = [center_coords_query[7],center_coords_query[6]]

    image_points = np.array([red_marker,
                    blue_marker,
                    green_marker,
                    yellow_marker], dtype= "double")
    
    succes, rvec, tvec = cv2.solvePnP_(object_points, image_points, k_instrinsic, dist_coef, flags = 0)

    print(object_points)
    print("rotation vector:", rvec)
    print("Translational vector:", tvec)
    return rvec, tvec

    
def image_acq_proc(lock, queue):
    
    print("im in image acq and processing")
    global query_img
    script_dir = os.path.dirname(__file__)
    rel_path = "Image"
    abs_file_path = os.path.join(script_dir, rel_path)
    picam2 = Picamera2()
    camera_config = picam2.create_still_configuration(main={"format": 'BGR888', "size": (3280, 2464)})
    picam2.configure(camera_config)
    picam2.start()
    center_coords_query = np.zeros((1, 8))
    while True:
        start = time.time()
        im_query = takePic(picam2, camera_config)
        #time.sleep(2)
        #im_reference = np.array(Image.open(str(abs_file_path)+"/Referenceimage.jpg"))
        #im_query = np.array(Image.open(str(abs_file_path)+"/Queryimage.jpg"))
        end = time.time()
        print(end-start)
        #cv2.imshow("im_query",im_reference)
        # if query_img == 0:
        #     center_point_reference, center_coords_ref = imageProc(im_reference)
        #     query_img += 1
        #     #time.sleep(1)
        #if query_img == 1:
        #time.sleep(1)
        start_image_proc = time.time()
        center_coords_query_value = imageProc(im_query)
        #print(center_coords_query_value)
        #print(center_coords_query.size)

        #print(center_coords_query)

        if isinstance(center_coords_query_value, int):
            image_data = center_coords_query
            lock.acquire()
            queue.put(image_data)
            lock.release()
            print(image_data)
            continue
        

        center_coords_query = center_coords_query_value

        print(center_coords_query)
        end_image_proc = time.time()
        print("image proc:",end_image_proc - start_image_proc)

        # calculateRotationTranslation(center_coords_ref,center_coords_query)
        image_data = center_coords_query[0]
        lock.acquire()
        queue.put(image_data)
        lock.release()
        # cv2.imshow("query", im_query)
        # cv2.waitKey(0)
        #calculateDifference(center_point_reference,center_point_query)

def comm(lock, queue):
    print("Im in comm function")
    #Instantiate server object instance
    server = TCPServer(host='192.168.0.102')
    image_data = None
    teensy = Teensy_comm("/dev/ttyACM0")
    xml = XMLhandler()
    while True:
        data=teensy.receive_send("command")
        if not isinstance(data, str):
            server.sendData(data)  # Receive data
            time.sleep(0.25)
        data_ekf = server.receiveData()
        if data_ekf is not None:
            data = teensy.receive_send(xml.process_xml(data_ekf).decode())
        if lock.acquire(block=False):
            if not queue.empty():
                image_data = np.array([queue.get()])
                lock.release()
                if isinstance(image_data, str): # If it is a string (An error)
                    continue
                else:
                    server.sendData(image_data)
        else:
            lock.release()
            continue


if __name__ == '__main__':
    Manager = multiprocessing.Manager()
    mutex = Lock()
    queue = SimpleQueue()
    p2 = multiprocessing.Process(target=comm, args=(mutex, queue))
    p2.start()
    p1 = multiprocessing.Process(target=image_acq_proc, args=(mutex, queue))
    p1.start()
    p1.join()
    p2.join()

