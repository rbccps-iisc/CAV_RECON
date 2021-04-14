import cv2
import rospy
from std_msgs.msg import String
# import geometry_msgs.msg
import numpy as np
# import segmentationobj
import time
import darknet
import darknet_images
import sys

thresh= 0.5 #object detection threshold
configPath = "./darknet_folders/yolo-obj_tiny.cfg"
weightPath = "./darknet_folders/yolov4-tiny.weights"
metaPath= "./darknet_folders/coco.data"
path='./logi2_cut.mp4'


#load the darknet yolov4 weights
network, class_names, class_colors = darknet.load_network(
    configPath,
    metaPath,
    weightPath,
    batch_size=1
)

def chk_obj(im_bgr):
    #function to run object detection and hence return if flagbox is True(obj present) else otherwise.

    imageret, detections = darknet_images.image_detection(
        im_bgr, network, class_names, class_colors, thresh
        )

    return imageret,detections


def FrameCapture(pub,path=None): 
      
    # Path to video file 
    vidObj = cv2.VideoCapture(0)
    print("fps of capture",vidObj.get(cv2.CAP_PROP_FPS)) #debug
    # Used as counter variable 
    counter=0
  
    # checks whether frames were extracted 
    success = 1

    while success and not rospy.is_shutdown():
        #for fps measurement
        start=time.time()
        #read the frames from video
        
        success, image = vidObj.read()
        image=image[0:1242,0:4416//2]
        #creating copy of the original image
        actual=image.copy()
        #variables for debug
        flag=0
        counter+=1
        seg_flag=False

        #resize the image
        image=cv2.resize(image,(640,480))
        
                #check if there is any obs in this segmented 2d roi,flagbox =True means there is a obstacle
                #if flagbox=False then there is no obs and continue to find the goal point
        try:
            imageret,detections=chk_obj(image)
            for dets in detections:
                if dets[0]=='person':

                    pub.publish('True')
                    flag=1
            if flag==0:

                # else:
                pub.publish('False')
               
            if len(detections)==0:
                pub.publish('False')

            
            # pub.publish('False')
            rate.sleep()
            cv2.imwrite('./res_obj/'+str(counter)+'.jpg',imageret)
                
                
        except Exception as e:
            print("Error",e)
        
        end=time.time()
        fps_label="here FPS: %.2f" % (1 / (end - start))
        print(fps_label,counter)


if __name__ == '__main__':
    try:
        
        
        pub = rospy.Publisher('/obj_det', String, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(1) # 10hz
        FrameCapture(pub,path)
    except Exception as e:
        print(e)
        
        sys.exit(0)
# FrameCapture(path)
