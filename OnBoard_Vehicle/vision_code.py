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
# path='./logi2_cut.mp4'
path='./imgslida_cam.avi'

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

def world_2d(points_ls):
    '''
    Convert a list of 3d points to corresponding image points using homography
    '''
    H=np.array([[11.387192116377845,-13.720570805787121,266.900926665664826],
    [9.653056228853927,0.322350790850648,2299.590518771450206],
    [0.038043554349025,0.002716870933722,1.000000000000000]])
    points_2d_ls = []
    for point in points_ls:
        point_3d = np.asarray(point)
        point_3d = point_3d.reshape(3,1)
        
        point_2d = np.dot(H,point_3d)
        point_2d = point_2d // point_2d[2,0]
        points_2d_ls.append(point_2d)
        
    return points_2d_ls

def FrameCapture(pub,path=None): 
      
    # Path to video file 
    vidObj = cv2.VideoCapture(0)
    # vidObj=cv2.VideoCapture(path)
    
    
    vidObj.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
    vidObj.set(cv2.CAP_PROP_FRAME_WIDTH,2560)

    print("fps of capture",vidObj.get(cv2.CAP_PROP_FPS)) #debug
    # Used as counter variable 
    counter=0

    success = 1
    points_ls=[[300,100,1],[300,-50,1],[500,-50,1],[500,100,1]]
    pointsmid=world_2d(points_ls)
    
    while success and not rospy.is_shutdown():
        #for fps measurement
        start=time.time()
        #read the frames from video
        
        success, image = vidObj.read()
        image=image[0:720,0:1280]
 
        #variables for debug
        flag=0
        counter+=1
        seg_flag=False

        #resize the image
        image=cv2.resize(image,(640,480))
        
        
        try:
            imageret,detections=chk_obj(image)
            
            cv2.line(image,(int(pointsmid[0][0]),int(pointsmid[0][1])),(int(pointsmid[3][0]),int(pointsmid[3][1])),(0,0,255), 2)
            cv2.line(image,(int(pointsmid[2][0]),int(pointsmid[2][1])),(int(pointsmid[1][0]),int(pointsmid[1][1])),(0,0,255), 2)
            cv2.line(image,(int(pointsmid[1][0]),int(pointsmid[1][1])),(int(pointsmid[0][0]),int(pointsmid[0][1])),(0,0,255), 2)
            cv2.line(image,(int(pointsmid[3][0]),int(pointsmid[3][1])),(int(pointsmid[2][0]),int(pointsmid[2][1])),(0,0,255), 2)
            
            flagbox=False
            
            for dets in detections:
                if dets[0]=='person':
                    x,y,w,h= dets[2]
                    
                    xmin = int(round(x - (w / 2)))
                    xmax = int(round(x + (w / 2)))
                    ymin = int(round(y - (h / 2)))
                    ymax = int(round(y + (h / 2)))

                    a=imageret.shape[0]
                    b=image.shape[0]
                    c=image.shape[1]

                    #the bounding box top left(xmin,ymin) and bottom right (xmax,ymax)
                    xmin=(c*xmin)//a
                    ymin=(b*ymin)//a
                    xmax=(c*xmax)//a 
                    ymax=(b*ymax)//a

                    cv2.rectangle(image,(xmin,ymin),(xmax,ymax),[255,0,0],3)

                    #the below code checks if the detected object is in the ROI in front of the robot
                    
                    roipoints=np.array([(pointsmid[0][0],pointsmid[0][1]),(pointsmid[1][0],pointsmid[1][1]),(pointsmid[2][0],pointsmid[2][1]),(pointsmid[3][0],pointsmid[3][1])],dtype=np.int32)
                    mask = np.zeros(image.shape[0:2], dtype=np.uint8)
                        
                    cv2.drawContours(mask, [roipoints], -1, (255, 255, 255), -1, cv2.LINE_AA)
                    
                    roipoints=np.array([(xmin,ymin),(xmin,ymax),(xmax,ymax),(xmax,ymin)],dtype=np.int32)
                    maskobj = np.zeros(image.shape[0:2], dtype=np.uint8)
                        # 
                    cv2.drawContours(maskobj, [roipoints], -1, (255, 255, 255), -1, cv2.LINE_AA)
                    #get the mask of the road roi and the mask of the detected obj(maskobj) and find the
                    #intersection between the two
                    res = cv2.bitwise_and(mask,maskobj)
                    
                    #chk if all pixels of the intersection are zero if yes that means the obj is not in 
                    #the roi
                    if cv2.countNonZero(res)!=0:
                        #if flagbox is true,then object is in the roi.
                        flagbox=True
                        flag=1
                        

                    pub.publish(str(flagbox))
                    
            if flag==0:

                # else:
                pub.publish('False')
               
            if len(detections)==0:
                pub.publish('False')

            # rate.sleep()
            
            cv2.putText(image,str(flag),(100,100), cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255), 2, cv2.LINE_AA)
            cv2.imwrite('./res_obj/'+str(counter)+'.jpg',image)
                
                
        except Exception as e:
            print("Error",e)
        
        end=time.time()
        fps_label="here FPS: %.2f" % (1 / (end - start))
        print(fps_label,counter)


if __name__ == '__main__':
    try:
        
        
        pub = rospy.Publisher('/obj_det', String, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        # rate = rospy.Rate(1) # 1hz
        FrameCapture(pub,path)
    except Exception as e:
        print(e)
        sys.exit(0)
