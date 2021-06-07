#external libraries
import cv2
import numpy as np
import time
from skimage import io, draw
import sys
import os
from configparser import ConfigParser
import json
from numba import jit
import natsort

#ros related libraries
import rosbag
import rospy
import message_filters
import ros_numpy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

#scripts
import segmentationobj
import darknet
import darknet_images


class pipeline:
    def __init__(self):
        #initializations
        self.config_object = ConfigParser()
        self.config_object.read("config.ini")

        #image related 
        self.image_config=self.config_object["Image"]
        self.webcam=int(self.image_config["Webcam"]) #the webcam number or camera number
        self.height=int(self.image_config["height"]) #the height and width of the image to be used for the pipeline.
        self.width=int(self.image_config["width"])


        #lidar related
        self.lidar_config=self.config_object["Lidar"]
        self.enable_lidar=int(self.lidar_config["Enable"]) #to enable or disable lidar-related distance computation.
        self.rotation_vector=json.loads(self.lidar_config["Rotation_matrix"]) # the lidar-camera calibration-rotation and translation matrices.
        self.translation_vector=json.loads(self.lidar_config["Translation_matrix"])
        self.K=json.loads(self.lidar_config["K"]) #camera intrinsics
        self.distortion_coeffs=json.loads(self.lidar_config["Distortion"]) #distortion coefficients of camera.

        #if recorded data for the camera is to be used
        self.recorded_cam=self.config_object["Recorded_images"] 
        self.rec_data=int(self.recorded_cam["recorded_data"]) #if camera_data is to be used from a recorded source.
        self.use_images=int(self.recorded_cam["use_images"]) #if recorded images need to be used.
        self.recorded_video_path=self.recorded_cam["recorded_video_path"] #if not recorded images then provide recorded video path.
        self.images_path=self.recorded_cam["images_path"] #recorded images path.

        #if recorded data for the lidar is to be used
        self.recorded_lidar=self.config_object["Recorded_lidar"] 
        self.recorded_lid_data=int(self.recorded_lidar["recorded_lid"]) #if recorded lidar data is to be used
        self.lidar_data_path=self.recorded_lidar["offline_data_path"] #recorded lidar data path,in the form of npy files,may or may not be time synced.

        #object detection related
        self.obj_config=self.config_object["Object_detection"]
        self.thresh=float(self.obj_config["threshold"]) #threshold for object detection
        self.configPath=self.obj_config["config_file_path"] #cfg file for object detection
        self.weightPath=self.obj_config["weights_file_path"] #trained model weight path
        self.metaPath=self.obj_config["meta_file_path"] #obj.data path

        #segmentation related
        self.seg_config=self.config_object["Segmentation"]
        self.homography=json.loads(self.seg_config["Homography_matrix"]) #homography matrix for 3d to 2d conversion
        self.roi_3d=json.loads(self.seg_config["3d_ROI"]) #3d region of interest eg from 4m to 12 m and +/- 5m sideways,the dimensions of the red box on road.

        #saving the results related
        self.results=self.config_object["Results_path"]
        self.save_results=self.results["Path"] #path to save results if needed.

        #display the results if needed
        self.display_config=self.config_object["Display_results"]
        self.display_show=int(self.display_config["Display"]) #if display the results,then aet it 1

        #to choose if both or any one of the models is to be loaded.
        self.models=self.config_object["models_needed"]

        #load the object detection model if needed
        if (self.models["both"])=="true" or self.models["one"]=="obj":
        	#flag if no_obj=0,this means obj detection model will be loaded.
            self.no_obj=0
            self.network, self.class_names,self.class_colors = darknet.load_network(
                self.configPath,
                self.metaPath,
                self.weightPath,
                batch_size=1
            )
        else:
            self.no_obj=1 
            print("No Object_detection,only segmentation")

        #to check if segmentation model is needed.
        if (self.models["both"])=="true" or self.models["one"]=="seg":
            #flag to indicate that segmentation model will be used
            self.no_seg=0
            #load segmentation model
            self.carla_utils_obj=self.load_seg_network()
        else:
        	#no segmentation model if no_seg=1
            self.no_seg=1
            print("No segmentation, only object detection")
            self.carla_utils_obj=None
        
        #if ros input 
        self.ros_decide=self.config_object["Ros_input"]
        self.ros=int(self.ros_decide["ROS"]) #if ros=1,then input for the pipeline comes from a bag file,maybe even live data can be inputted similarly.
        #variables to be used across different functions
        self.counter=0 #to count the number of images processed in the pipeline.
        self.start=0 #to start timer to measure fps
        

    def callback(self,image,velodyne):
        #function to extract synced lidar and camera data,when called.
        if self.counter==0:
        	print("timer started")
        	self.start=time.time()
        #time extracted from image and lidar messages,just to see how well they are in sync,by computing the difference in their times or storing it in folder.
        imgt=str(float(image.header.stamp.secs)+(float(image.header.stamp.nsecs)/(10**9)))
        lidt=str(float(velodyne.header.stamp.secs)+(float(velodyne.header.stamp.nsecs)/(10**9)))

        imgt=str(round(float(imgt),2))
        lidt=str(round(float(lidt),2))
        # print(imgt,lidt)
        
        #each time the extracted point cloud data which is in time sync with the image,is send to next function,process
        pcds_list = ros_numpy.point_cloud2.pointcloud2_to_array(velodyne)
        image = np.frombuffer(image.data, dtype=np.uint8).reshape(480,640, -1)
        
        
        self.process(pcds_list,image)

    def callback_cam(self,image):
    	#function to be used when there is no lidar data and only camera data is there.
        if self.counter==0:
        	print("timer started")
        	self.start=time.time()
        #each time extract the image and send it to process	function
        image = np.frombuffer(image.data, dtype=np.uint8).reshape(480,640, -1)
        pcds_list=[]
        self.process(pcds_list,image)

    def listener(self,image_color, velodyne_points):
    	#function to be called when ros is used and lidar camera topics are published
        rospy.init_node('calibrate_camera_lidar', anonymous=True)

        #image topic is subscribed
        image_sub = message_filters.Subscriber(image_color, Image)
	    
	    #lidar topic is subscribed
        velodyne_sub = message_filters.Subscriber(velodyne_points, PointCloud2)
        
        #the topics are time synced,slop decides the resolution/fineness of the sync,such as if slop=0.01 then the data should  be tim-synced within 10ms.
        #prefer to keep queue size=lowest frequency among the sensors,here usually its the lidar.
        ats = message_filters.ApproximateTimeSynchronizer(
	        [image_sub,velodyne_sub], queue_size=10, slop=0.1)

        #callback function when you get some lidar frame corresponding to camera data within the given slop.
        aa=ats.registerCallback(self.callback)
        
       	
	    # Keep python from exiting until this node is stopped
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo('Shutting down')

    def listener_cam_only(self,image_color):
    	#function called when ros is used and only camera input is there,no lidar data is there for input.or is disabled.
        rospy.init_node('camera_only', anonymous=True)
        #subscriber to the image data and callback_cam is the callback function when camera data is recieved.
        sub=rospy.Subscriber(image_color,Image,self.callback_cam)

        #keep python from exiting until this node is stopped
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
        	rospy.loginfo('Shutting down')

    def load_seg_network(self):
        #funtion to load the segmentation model
        carla_utils_obj_load = segmentationobj.carla_utils()
        return carla_utils_obj_load

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

        #if true then its a valid segmented roi(the red box drawn on road),else no valid segmented roi was found
        if(np.array_equal(seg_img,cv2.bitwise_and(seg_img_cpy,seg_img_cpy_1)) == True):
            return True
        else:
            return False


    def chk_obj(self,im_bgr):
        #function to run object detection
        imageret, detections = darknet_images.image_detection(
            im_bgr, self.network, self.class_names, self.class_colors, self.thresh
            )
        #the image to be returned(as the detections are acc to dimensions of this image),and the detections for that particular image.
        return imageret,detections


    def _2d_to_3d(self,goal):
        #function to convert 2d point to 3d goal point using homography matrix
        point=[goal[1],goal[0],1]

        mat=np.matrix(self.homography)
        
        matinv=mat.I

        
        prod=np.dot(matinv,point)
        scalar=prod[0,2]
        #the 3d point is returned
        return [(prod[0,0]/scalar),(prod[0,1]/scalar)]

   
    def getdepth(self,projected_head_pose_box_points,pt1,pt2,points3dnew):
        #function to find depth of the particular bounding box.

        #restrict points which fall in the given bounding box 
        inrange = np.where((projected_head_pose_box_points[:, 0][:,0] >= pt1[0]) &
                               (projected_head_pose_box_points[:, 0][:,0] < pt2[0]) &
                               (projected_head_pose_box_points[:, 0][:,1] >= pt1[1]) &
                               (projected_head_pose_box_points[:, 0][:,1] < pt2[1]))

        
        points3d=points3dnew[inrange[0]]
        
        depth_arr=[]
        histo={}
        #find the depth of the bounding box as the max of the histogram.
        #TODO-find the histogram only in the middle region of the bounding box(bb),like 25 pixels around center of bb,so that this depth is more accurate.
        for jj in range(len(points3d)):
            
            depth = np.sqrt(points3d[jj, 0]**2+points3d[jj,1]**2+points3d[jj,2]**2)
            depth_arr.append(depth)

        for ii in depth_arr:
    
            histo[round(ii,1)]=histo.get(round(ii,1),0)+1

        dd={k: v for k, v in sorted(histo.items(), key=lambda item: item[1])}
        neededdepth = max(dd, key=dd.get) #the depth of the bounding box
        

        return neededdepth

    def FrameCapture(self): 
        # funtion to be used if ros is not used as input for camera data.It is used to check if the camera data is accessible from any of the sources provided
        self.start=time.time()
        #initial state of the variables.
        success=False
        images_list=[] #to store list of images in case the recorded images are to be used.
        #check if we need to use the recorded data,it can be video or images.
        if self.rec_data==1:
            print("Using recorded data,either video or images")
            #if use_images=1 meaning use recorded images
            if self.use_images==1:
            	print("Using images from the path specified")
            	images_list=natsort.natsorted(os.listdir(self.images_path))
            	vidObj=None
            	success=True

            else:
            	#use recorded video if use_images=0
                print("Opening video file path")
                vidObj = cv2.VideoCapture(self.recorded_video_path)
                success, image = vidObj.read()

        else:
        	#if recorded data is not to be used then use webcam live feed or any camera which is connected.
            web=self.webcam
            vidObj = cv2.VideoCapture(web)
            success, image = vidObj.read()

        if not success:
            print("Unable to get feed from camera or path given")
            #TODO : Retry for some time,depending from where you get the feed,warn the remote end
            sys.exit(0)
        
        #using recorded lidar data in any of the above cases,because ros is disabled.
        if self.enable_lidar==1:
        	#lidar files list
        	pcds_list=natsort.natsorted(os.listdir(self.lidar_data_path))
        else:
        	pcds_list=[]
        #call function to repeat the pipeline for every image or image-lidar pair.
       	self.repeat(success,pcds_list,images_list,vidObj)

    def repeat(self,success,pcds_list,images_list,vidObj):
    	#function repeats for every input data/pair and calls process funtion to do the pipeline execution on each input.
        while success:

            if vidObj!=None:
            	#if live data or recorded video is used then enter this condition.
                success, image = vidObj.read()
            else:

                image=cv2.imread(self.images_path+str(images_list[self.counter]))
                #condition to exit,when the number of images have been exhausted in the path provided.
                if self.counter<len(images_list):
                    success=True
                else:
                    success=False
            #to execute the rest of the pipeline call the below function.
            self.process(pcds_list,image)
            
            
    def process(self,pcds_list,image):

        #function to perform object detection,estimate distance of the object,segment the road and give a goal point if possible.
        #Any one of these functionalities can be used at a given time,depending on the config file settings.
        
        image=cv2.resize(image,(self.width,self.height)) #resize the image according to the dimensions in the cfg file.

        #if lidar data is to be used indicated by enable_lidar.
        if self.enable_lidar==1:
        	#if recorded lidar data is to be used or is it to be used from ROS.
	        if self.recorded_lid_data!=0 and self.ros==0:
	        	#recorded lidar data is used.
	        	points3dnew=np.array(np.load(self.lidar_data_path+str(pcds_list[self.counter])).tolist(),dtype='float32')
	        else:
	        	#live data from ros or rosbag is used.
	        	points3dnew=np.array(pcds_list.tolist(),dtype='float32')
	 
	        
	        #from here it is same for live or offline lidar data
	        #try to reverse the x,y coordinates and their signs to a format accepted by cv2.projectpoints.Because the data recorded by ros cant be used
	        # directly as it is,it needs to be converted to proper coordinate system.This is not needed always,in case lidar data is recorded by some other means.
	        temp=points3dnew[:,0].copy()
	        points3dnew[:,0]=-points3dnew[:,1]
	        points3dnew[:,1]=temp

	        #filter points which are only in front of the camera and not behind the camera.
	        inrange = np.where((points3dnew[:, 1] > 0))

	        points3dnew= points3dnew[inrange[0]]
	        points3dnew=np.array(points3dnew[:,:3],dtype='float32')
	        #project the 3d lidar points on the image, i.e convert it to 2d points.
	        self.rotation_vector=np.asarray(self.rotation_vector)
	        self.translation_vector=np.asarray(self.translation_vector)
	        self.camera_matrix=np.array(self.K).reshape(3, 3).astype(np.float32)
	        self.distortion_coeffs=np.array(self.distortion_coeffs).reshape(5, 1).astype(np.float32)
	        #the array obtained below,is the 2d points corresponding to the 3d lidar points.
	        projected_head_pose_box_points, _ = cv2.projectPoints(points3dnew,
	                                                                self.rotation_vector,
	                                                                self.translation_vector,
	                                                                self.camera_matrix,
	                                                                self.distortion_coeffs)


	    #3d region of interest(roi) as specified in the config file
        points_ls=self.roi_3d
        
        #some variables declared as flags,needed for some checks later in the code
        flag=0
        self.counter+=1 # a counter to save images
        seg_flag=False #to indicate if segmentation was succesful or no
        flagbox=False #to indicate if object is present in the scene or no 
        
        try:
            #if segmentation model is needed as specified in config file,then this code gets executed.
            #get the segmented road 2d roi(region of interest) corresponding to the 3d roi as defined above.
            if self.no_seg==0:
                pts_2d_ls,res_img=self.carla_utils_obj.run_seg(image,points_ls)
                #to check if the segmented 2d region is valid,if seg_flag=True then its valid,else not-valid.
                if len(pts_2d_ls)!=0:
                    seg_flag=self.chk_parking_spot(pts_2d_ls,res_img)

        except Exception as e:
            print("segmentation_exception ",e)
            #TODO:send some warning, instead of just exit
            sys.exit(0)
        
        
        #the below snippet is executed only if object detection model is needed(as specified in config file)
        if self.no_obj==0:
        	#to check if there is any valid segmentaion then use the output image from segmented region else use res_image as image(the input image directly).
            if seg_flag==False:
                res_img=image

            #run the object detection network on each image and get the detections and image
            imageret,detections=self.chk_obj(res_img)

            imagePath=res_img
            count_for_obs=0 #count the number of objects detected.
            #loop through each detection
            for detection in detections:

                #for each detection get the confidence and compute the bounding box
                label = detection[0]
                confidence = detection[1]
                
                #computing the bounding box
                bounds = detection[2]
                x,y,w,h=bounds
                xmin = int(round(x - (w / 2)))
                xmax = int(round(x + (w / 2)))
                ymin = int(round(y - (h / 2)))
                ymax = int(round(y + (h / 2)))
                a=imageret.shape[0]
                b=imagePath.shape[0]
                c=imagePath.shape[1]

                #the bounding box top left(xmin,ymin) and bottom right (xmax,ymax)
                xmin=(c*xmin)//a
                ymin=(b*ymin)//a
                xmax=(c*xmax)//a 
                ymax=(b*ymax)//a

                #for lidar
                xmid=(xmin+xmax)/2
                ymid=(ymax+ymin)/2
                pt1=[int(xmid-25),int(ymid-25)]
                pt2=[int(xmid+25),int(ymid+25)]
                
                #find the depth if lidar is enabled,else depth is dummy=0m
                if self.enable_lidar==1:
                	neededdepth=self.getdepth(projected_head_pose_box_points,pt1,pt2,points3dnew)
                else:
                	neededdepth=0

                flagbox=True #meaning a object is detected in the scene

                #draw the bounding box on the object if needed
                if len(str(self.save_results))!=0 or self.display_show==1:
                    if neededdepth>10 or self.enable_lidar==0:
                        cv2.rectangle(imagePath,(xmin,ymin),(xmax,ymax),[255,0,0],3)
                    elif self.enable_lidar==1 and neededdepth<10:
                    	#enter this only if obj is close(<10m)
                        cv2.rectangle(imagePath,(xmin,ymin),(xmax,ymax),[0,0,255],3)
                        count_for_obs+=1
                        #alert as to how many obstacles are close.
                        cv2.putText(imagePath, 'ALERT-'+str(count_for_obs),(300,50), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0,0,255), 2, lineType=cv2.LINE_AA)

                #put the depth on the image.    
                    cv2.putText(imagePath, str(int(neededdepth))+'m', (xmin, ymin-5), cv2.FONT_HERSHEY_SIMPLEX,
                1, (0,255,0), 2, lineType=cv2.LINE_AA)

                
                #alert driver based on distance,on terminal 
                #TODO:put it as a topic later or as needed by the remote operator
                if neededdepth>0 and neededdepth<10 and (len(str(self.save_results))==0 or self.display_show==0):
                    count_for_obs+=1
                    print("Obstacle near,Alert!!",count_for_obs)
                    

                #if segmentation model is also loaded,then check if the detected object is in the segmented ROI.
                #here if there is a valid segmented roi(seg_flag=True) then see if the object is in this roi,hence before checking put flagbox=False,
                #make it true when the detected obj(if any) lies in the roi.
                if seg_flag==True:
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
                        
                        pointsmid=pts_2d_ls #the 4 2d points surrounding the 2d roi on road.
                        
                        roipoints=np.array([(pointsmid[0][1],pointsmid[0][0]),(pointsmid[1][1],pointsmid[1][0]),(pointsmid[2][1],pointsmid[2][0]),(pointsmid[3][1],pointsmid[3][0])],dtype=np.int32)
                        mask = np.zeros(imagePath.shape[0:2], dtype=np.uint8)
                        
                        cv2.drawContours(mask, [roipoints], -1, (255, 255, 255), -1, cv2.LINE_AA)

                        roipoints=np.array([(xmin,ymin),(xmin,ymax),(xmax,ymax),(xmax,ymin)],dtype=np.int32)
                        maskobj = np.zeros(imagePath.shape[0:2], dtype=np.uint8)
                        
                        cv2.drawContours(maskobj, [roipoints], -1, (255, 255, 255), -1, cv2.LINE_AA)
                        #get the mask of the road roi and the mask of the detected obj(maskobj) and find the
                        #intersection between the two
                        res = cv2.bitwise_and(mask,maskobj)
                      
                        #chk if all pixels of the intersection are zero if yes that means the obj is not in 
                        #the roi
                        if cv2.countNonZero(res)!=0:
                            #break even if one obj is in the roi,flagbox=True indicates there is at least one obj detected in the roi.
                            flagbox=True
                            break 
        
            
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
            parkinggoal2d=self.carla_utils_obj.find_midpoint(ptall)
            parkinggoal3d=self._2d_to_3d(parkinggoal2d)
            
            flag=1 #just a variable t indicate that this particular condition is True

            #if specified in config,then save the results
            if len(str(self.save_results))!=0 or self.display_show==1:

                cv2.circle(imagePath,(int(parkinggoal2d[1]),int(parkinggoal2d[0])), 8, (0,255,255), -1)

                #printing the 3d goal point on to the image
                imagePath=cv2.putText(imagePath,"goal:"+"X:"+str(round((parkinggoal3d[0]/100),2))+",Y:"+str(round(parkinggoal3d[1]/100,2)), (100,80), cv2.FONT_HERSHEY_SIMPLEX,1, 
                (0,255,0), 2, cv2.LINE_AA)
                # save the image
            if len(str(self.save_results))!=0:
                cv2.imwrite(str(self.save_results)+str(self.counter)+'.jpg',imagePath)
            #display the results if specified in cfg file
            if self.display_show==1:
                cv2.imshow('result',imagePath)
                if cv2.waitKey(100) & 0xFF == ord('q'):
                    return
                    
           
        #if the object is detected in the segmented roi
        elif seg_flag==True and flagbox==True:
            
            aa="obj_Det_in"
            flag=1

            #if specified in config,then save the results
            if len(str(self.save_results))!=0 or self.display_show==1:
                imagePath=cv2.putText(imagePath,aa, (300,80), cv2.FONT_HERSHEY_SIMPLEX,1, 
                (0,0,255), 2, cv2.LINE_AA)
            #save and display results if needed    
            if len(str(self.save_results))!=0:
                cv2.imwrite(str(self.save_results)+str(self.counter)+'.jpg',imagePath)
            if self.display_show==1:
                cv2.imshow('result',imagePath)
                if cv2.waitKey(100) & 0xFF == ord('q'): #its 100 and not 1 cause the program was exiting if the waitkey is less
                    return 

        #if there is no object detected and also either segmentation failed or is not acrivated only
        elif flagbox==False and seg_flag==False:
            aa="no_detection"
            flag=1
            imagePath=image #because in somecases imagePath would not have been assigned hence we assign it to original image.

            #if specified in config,then save the results or display them.
            if len(str(self.save_results))!=0 or self.display_show==1:
                imagePath=cv2.putText(imagePath,aa, (300,80), cv2.FONT_HERSHEY_SIMPLEX,1, 
                (0,0,255), 2, cv2.LINE_AA)

            if len(str(self.save_results))!=0:
                cv2.imwrite(str(self.save_results)+str(self.counter)+'.jpg',imagePath)
            
            if self.display_show==1:
                cv2.imshow('result',imagePath)
                if cv2.waitKey(100) & 0xFF == ord('q'):
                    return
                

        #if object is detected,but segmentation is not active
        elif seg_flag==False and flagbox==True:
            
            aa="obj_Det_only"

            flag=1
            #if specified in config,then save the results or display it.
            if len(str(self.save_results))!=0 or self.display_show==1:
                imagePath=cv2.putText(imagePath,aa, (300,80), cv2.FONT_HERSHEY_SIMPLEX,1, 
                (0,0,255), 2, cv2.LINE_AA)

            if len(str(self.save_results))!=0:
                cv2.imwrite(str(self.save_results)+str(self.counter)+'.jpg',imagePath)
            
            if self.display_show==1:
            
                cv2.imshow('result',imagePath)
                
                if cv2.waitKey(100) & 0xFF == ord('q'):
                    
                    return
                
                
        
        #to get fps for each run and at total time for which the pipeline has been running.
        end=time.time()

        print("count",self.counter)
        print("fps",self.counter/(end-self.start))
        print("total_time",round(end-self.start,3))
       



if __name__ == '__main__':
    try:
        pipe=pipeline() #pipe is object of class pipeline

        image_color = '/image_raw' #image topic to be subscribed to.
        #to check if ros is to be used and if lidar data is to be used.
        if pipe.ros==1 and pipe.enable_lidar==1:
                 
            velodyne_points = '/velodyne_points' #lidar topic to be subscribed to if lidar data is enabled.

            pipe.listener(image_color, velodyne_points)
        #if ros and only camera data is to be used with ros.
        elif pipe.ros==1 and pipe.enable_lidar==0:
        	pipe.listener_cam_only(image_color)

        #if ros is not be used for input data.
        else:
            pipe.FrameCapture() #calling the member function, to start the pipeline
        #destroy window if any data was being displayed.    
        cv2.destroyAllWindows()
    except KeyboardInterrupt:        
        sys.exit(0)

