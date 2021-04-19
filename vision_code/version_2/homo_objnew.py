#external libraries
import cv2
import numpy as np
import time
from skimage import io, draw
import sys
import os
from configparser import ConfigParser
import json

#scripts
import segmentationobj
import darknetTR


class pipeline:
    def __init__(self):
        #initializations
        self.config_object = ConfigParser()
        self.config_object.read("config.ini")

        #image related 
        self.image_config=self.config_object["Image"]
        self.path=self.image_config["Recorded_video_path"]
        self.webcam=int(self.image_config["Webcam"])
        self.height=int(self.image_config["height"])
        self.width=int(self.image_config["width"])

        #object detection related
        self.obj_config=self.config_object["Object_detection"]
        self.conf_thresh=float(self.obj_config["threshold"])
        self.input_size_for_darknet=int(self.obj_config["input_size_for_darknet"])
        self.weight_file=self.obj_config["weights_file_path"]
        self.metaPath=self.obj_config["meta_file_path"]
        self.nms=float(self.obj_config["nms"])

        #segmentation related
        self.seg_config=self.config_object["Segmentation"]
        self.homography=json.loads(self.seg_config["Homography_matrix"])
        self.roi_3d=json.loads(self.seg_config["3d_ROI"])

        #saving the results related
        self.results=self.config_object["Results_path"]
        self.save_results=self.results["Path"]

        #to choose if both or any one of the models is to be loaded.
        self.models=self.config_object["models_needed"]

        #load the object detection model if needed
        if (self.models["both"])=="true" or self.models["one"]=="obj":
            self.no_obj=0
            self.forobj=darknetTR.YOLO4RT(input_size=self.input_size_for_darknet,
                 weight_file=self.weight_file,
                 metaPath=self.metaPath,
                 nms=self.nms,
                 conf_thres=self.conf_thresh,
                 device='cuda')
            
        else:
            self.no_obj=1
            print("No Object_detection,only segmentation")

    def load_seg_network(self):
        #funtion to load the segmentation model
        carla_utils_obj = segmentationobj.carla_utils()
        return carla_utils_obj

    def chk_parking_spot(self,parking_spot_edges,seg_img):
        '''
        Check if all the pixels within the potential parking spot correspond to road pixels
        parking_spot_edge: Edges of the potential parking spot
        seg_img: Segmented image
        '''
        seg_img_cpy = np.copy(seg_img)
        seg_img_cpy_1 = np.copy(seg_img)
        #Convert into OpenCV format
        parking_spot_edges = [element[::-1] for element in parking_spot_edges]
        seg_cnt = [np.array([parking_spot_edges[0],parking_spot_edges[1],parking_spot_edges[2],\
                               parking_spot_edges[3]],dtype=np.int32)]
        # cv2.drawContours(seg_img, seg_region, -1, (0,255,0), 3)
        cv2.fillPoly(seg_img_cpy, pts = seg_cnt, color=(255,255,255))
        if(np.array_equal(seg_img,cv2.bitwise_and(seg_img_cpy,seg_img_cpy_1)) == True):
            return True
        else:
            return False



    def _2d_to_3d(self,goal):
        #function to convert 2d point to 3d goal point using homography matrix
        point=[goal[1],goal[0],1]

        mat=np.matrix(self.homography)
        
        matinv=mat.I

        
        prod=np.dot(matinv,point)
        scalar=prod[0,2]
        return [(prod[0,0]/scalar),(prod[0,1]/scalar)]

    def FrameCapture(self): 
        
        #if no path for the video file is specified,open the webcam 
        #TODO: change according to interface eg- ROS,etc
        if len(self.path)==0:
            print("Opening webcam,no file path specified")
            web=self.webcam
            vidObj = cv2.VideoCapture(web)

        #open the video file path
        else:
            print("Opening video file path")
            vidObj = cv2.VideoCapture(self.path)
    
        
        #check if segmentation model is to be loaded as specified in the config file
        if (self.models["both"])=="true" or self.models["one"]=="seg":
            #flag to indicate that segmentation model will be used
            no_seg=0
            #load segmentation model
            carla_utils_obj=self.load_seg_network()
        else:
            no_seg=1
            print("No segmentation, only object detection")

        # Used as counter variable 
        counter=0
        # checks whether frames were extracted 
        success, image = vidObj.read()
        if not success:
            print("Unable to get feed from camera or path given")
            #TODO : Retry for some time,depending from where you get the feed,warn the remote end
            sys.exit(0)

        while success:

            #for fps measurement
            start=time.time()
            #read the frames from video
            success, image = vidObj.read()
            
            #some variables declared as flags,needed for some checks later in the code
            flag=0
            counter+=1 # a counter to save images
            seg_flag=False #to indicate if segmentation was succesful or no
            flagbox=False #to indicate if object is present in the scene or no 
            

            #resize the image
            image=cv2.resize(image,(self.width,self.height))

            #3d region of interest(roi) as specified in the config file
            points_ls=self.roi_3d
            
            try:
                #if segmentation model is needed as specified in config file,then this code gets executed.
                #get the segmented road 2d roi(region of interest) corresponding to the 3d roi as defined above.
                if no_seg==0:
                    pts_2d_ls,res_img=carla_utils_obj.run_seg(image,points_ls)
                    
                    if len(pts_2d_ls)!=0:
                        seg_flag=self.chk_parking_spot(pts_2d_ls,res_img)

            except Exception as e:
                print("segmentation_exception ",e)
                #TODO:send some warning, instead of just exit
                sys.exit(0)

            
            #the below snippet is executed only if object detection model is needed(as specified in config file)
            if self.no_obj==0:

                if seg_flag==False:
                    res_img=image

                #run the object detection network on each image and get the detections and image
                
                detections=self.forobj.detect(res_img)

                imagePath=res_img

                #loop through each detection
                for kk in range(len(detections)):

                    classname=str(detections[kk][0])

                    score=str(round(detections[kk][1],2))
                    aa=int(detections[kk][2][0])
                    bb=int(detections[kk][2][1])
                    cc=int(detections[kk][2][2])+int(detections[kk][2][0])
                    dd=int(detections[kk][2][3])+int(detections[kk][2][1])

                    #get the xmin,ymin,xmax,ymax coordinates of the bounding box
                    xmin=(aa*res_img.shape[1])//416
                    ymin=(bb*res_img.shape[0])//416
                    xmax=(cc*res_img.shape[1])//416
                    ymax=(dd*res_img.shape[0])//416
                    
                    flagbox=True #meaning a object is detected in the scene

                    #draw the bounding box on the object if needed
                    if len(str(self.save_results))!=0:
                        cv2.rectangle(imagePath,(xmin,ymin),(xmax,ymax),[255,0,0],3)

                    #if segmentation model is also loaded,then check if the detected object is in the segmented ROI.
                    if no_seg==0:
                        flagbox=False
                        
                        #checks the overlap between the detected object and roi on the road.
                        if len(pts_2d_ls)!=0 and seg_flag==True:

                            #to find iou between the bounding box of the object and the the segmented road 2d roi(region of interest)
                            #uncomment the below code and comment iou=1
                            # xa=max(xmin,pointsmid[0][1])
                            # ya=max(ymin,pointsmid[0][0])
                            # xb=min(xmax,pointsmid[2][1])
                            # yb=min(ymax,pointsmid[2][0])
                            # interArea = max(0, (xb - xa + 1)) * max(0, (yb - ya + 1))
                            # boxAArea = (xmax - xmin + 1) * (ymax- ymin + 1)
                            # boxBArea = (pointsmid[0][1] - pointsmid[2][1] + 1) * (pointsmid[0][0] - pointsmid[2][0] + 1)
                            # iou = interArea / float(boxAArea + boxBArea - interArea)
                            
                            pointsmid=pts_2d_ls
                            
                            roipoints=np.array([(pointsmid[0][1],pointsmid[0][0]),(pointsmid[1][1],pointsmid[1][0]),(pointsmid[2][1],pointsmid[2][0]),(pointsmid[3][1],pointsmid[3][0])],dtype=np.int32)
                            mask = np.zeros(imagePath.shape[0:2], dtype=np.uint8)
                            
                            cv2.drawContours(mask, [roipoints], -1, (255, 255, 255), -1, cv2.LINE_AA)

                            roipoints=np.array([(xmin,ymin),(xmin,ymax),(xmax,ymax),(xmax,ymin)],dtype=np.int32)
                            maskobj = np.zeros(imagePath.shape[0:2], dtype=np.uint8)
                            # 
                            cv2.drawContours(maskobj, [roipoints], -1, (255, 255, 255), -1, cv2.LINE_AA)
                            #get the mask of the road roi and the mask of the detected obj(maskobj) and find the
                            #intersection between the two
                            res = cv2.bitwise_and(mask,maskobj)
                          
                            #chk if all pixels of the intersection are zero if yes that means the obj is not in 
                            #the roi
                            if cv2.countNonZero(res)!=0:
                                
                                # counter+=1
                                flagbox=True
                                break #############################check
            
                 
            #if there is no object in the segmented roi,find the 3d goal point.
            if flagbox==False and seg_flag==True:
                
                #TODO:OPTIMIZE 
                #below snippet is to find the 3d goal point
                imagePath=res_img
                pt1=[pts_2d_ls[0][1],pts_2d_ls[0][0]]
                pt2=[(pts_2d_ls[0][1]+pts_2d_ls[1][1])//2,pts_2d_ls[0][0]]
                pt4=[(pts_2d_ls[0][1]+pts_2d_ls[3][1])//2,(pts_2d_ls[0][0]+pts_2d_ls[3][0])//2]
                ptintermediate1=[(pts_2d_ls[0][1]+pts_2d_ls[1][1])//2,(pts_2d_ls[0][0]+pts_2d_ls[1][0])//2]
                ptintermediate2=[(pts_2d_ls[3][1]+pts_2d_ls[2][1])//2,(pts_2d_ls[3][0]+pts_2d_ls[2][0])//2]
                pt3=[(ptintermediate1[0]+ptintermediate2[0])//2,(ptintermediate1[1]+ptintermediate2[1])//2]
               
                ptall=np.array([pt1[::-1],pt2[::-1],pt3[::-1],pt4[::-1]],dtype=int)

                #parkinggoal2d is the 2d goal point
                parkinggoal2d=carla_utils_obj.find_midpoint(ptall)
                parkinggoal3d=self._2d_to_3d(parkinggoal2d)
                
                flag=1

                #if specified in config,then save the results
                if len(str(self.save_results))!=0:

                    cv2.circle(imagePath,(int(parkinggoal2d[1]),int(parkinggoal2d[0])), 8, (0,255,255), -1)

                    #printing the 3d goal point on to the image
                    imagePath=cv2.putText(imagePath,"goal:"+"X:"+str(round((parkinggoal3d[0]/100),2))+",Y:"+str(round(parkinggoal3d[1]/100,2)), (100,80), cv2.FONT_HERSHEY_SIMPLEX,1, 
                    (0,0,255), 2, cv2.LINE_AA)
                    # save the image
                    cv2.imwrite(str(self.save_results)+str(counter)+'.jpg',imagePath)
                
            #if the object is detected in the segmented roi
            elif seg_flag==True and flagbox==True:
                
                aa="obj_Det"
                flag=1
                #if specified in config,then save the results
                if len(str(self.save_results))!=0:
                    imagePath=cv2.putText(imagePath,aa, (300,80), cv2.FONT_HERSHEY_SIMPLEX,1, 
                    (0,0,255), 2, cv2.LINE_AA)
                    
                    cv2.imwrite(str(self.save_results)+str(counter)+'.jpg',imagePath)

            #if there is no object detected and also either segmentation failed or is not acrivated only
            elif flagbox==False and seg_flag==False:
                aa="no_road_detected"
                flag=1
                imagePath=image
                #if specified in config,then save the results
                if len(str(self.save_results))!=0:
                    imagePath=cv2.putText(imagePath,aa, (300,80), cv2.FONT_HERSHEY_SIMPLEX,1, 
                    (0,0,255), 2, cv2.LINE_AA)
                    
                    cv2.imwrite(str(self.save_results)+str(counter)+'.jpg',imagePath)

            #if object is detected,but segmentation is not active
            elif seg_flag==False and flagbox==True:
                
                aa="obj_Det"

                flag=1
                #if specified in config,then save the results
                if len(str(self.save_results))!=0:
                    imagePath=cv2.putText(imagePath,aa, (300,80), cv2.FONT_HERSHEY_SIMPLEX,1, 
                    (0,0,255), 2, cv2.LINE_AA)
                    
                    cv2.imwrite(str(self.save_results)+str(counter)+'.jpg',imagePath)
            
            #to get fps for each run
            end=time.time()
            fps_label="here FPS: %.2f" % (1 / (end - start))
            print(fps_label)



if __name__ == '__main__':
    try:
        pipe=pipeline() #pipe is object of class pipeline
        pipe.FrameCapture() #calling the member function, to start the pipeline
    except KeyboardInterrupt:
        
        sys.exit(0)

