import glob
import numpy as np
import cv2 
import os
import time
from statistics import mean
import pickle
import torch
import torchvision.transforms as transforms
from PIL import Image
import sys
from configparser import ConfigParser
import json
#scripts
from auto_park_utils import auto_park_vision

def display_image(winname,img):
    cv2.namedWindow(winname,cv2.WINDOW_NORMAL)
    cv2.imshow(winname,img)
    key = cv2.waitKey(0)
    if(key & 0xFF == ord('q')):
        cv2.destroyAllWindows()
        sys.exit(0)


def ret_line_eq(pt1,pt2):
    '''
    Returns m1,c1 given 2 points
    '''
    points = [pt1,pt2]
    x_coords, y_coords = zip(*points)
    A = np.vstack([x_coords,np.ones(len(x_coords))]).T
    m1, c1 = np.linalg.lstsq(A, y_coords)[0] #TODO: Improve this, it computes the least square solution
    return m1,c1


def pot_parking_spot(orig_img,inf_img,points_2d_ls,save_results):
    '''
    Function to detect potential parking spot ,i.e 2d roi which is on the road only
    '''
    #TODO: Optimize the code
    xcoords_ls = []
    ycoords_ls = []
    for point in points_2d_ls:
        xcoords_ls.append(point[0][0])
        ycoords_ls.append(point[1][0])
    
    #Line equations of the top and bottom line
    coeff_top = np.polyfit(xcoords_ls[0:2],ycoords_ls[0:2],1)
    line_top = np.poly1d(coeff_top)

    coeff_bottom = np.polyfit(xcoords_ls[2:4],ycoords_ls[2:4],1)
    line_bottom = np.poly1d(coeff_bottom)
    
    flag_top = 0
    flag_bottom = 0
    #Points for potential parking spot
    pt_tl = []
    pt_tr = []
    pt_bl = []
    pt_br = []
    for x in range(0,inf_img.shape[1]):
        
        if(inf_img[int(line_top(x)),x] == 255 and flag_top == 0):
            pt_tl = [int(line_top(x)),x]
            pt_tr = [int(line_top(x+250)),int(x+250)]
            
            
            flag_top = 1

        if(inf_img[int(line_bottom(x)),x] == 255 and flag_bottom == 0):
            pt_bl = [int(line_bottom(x)),x]
            pt_br = [int(line_bottom(x+250)),int(x+250)]
            
            
            flag_bottom = 1
        
        if(flag_top == 1 and flag_bottom ==1):
            
            break

    if(flag_top == 1 and flag_bottom == 1):
        if len(str(save_results))!=0:

            #for debug
            cv2.line(orig_img,(pt_tl[1],pt_tl[0]),(pt_tr[1],pt_tr[0]),(0,0,255),2)
            cv2.line(orig_img,(pt_tr[1],pt_tr[0]),(pt_br[1],pt_br[0]),(0,0,255),2)
            cv2.line(orig_img,(pt_br[1],pt_br[0]),(pt_bl[1],pt_bl[0]),(0,0,255),2)
            cv2.line(orig_img,(pt_bl[1],pt_bl[0]),(pt_tl[1],pt_tl[0]),(0,0,255),2)
    
    pt_ls = [pt_bl,pt_br,pt_tr,pt_tl]
    return orig_img,pt_ls


class carla_utils():

    def __init__(self):
        
        self.auto_park_obj = auto_park_vision()
        self.ext_mat = None
        self.points_2d_ls = None
        self.window_width = 1280
        self.window_height = 720
        self.seg_img = None
        self.rect_pts = None
        self.count=0
        self.config_object = ConfigParser()
        self.config_object.read("config.ini")
        self.seg_config=self.config_object["Segmentation"]
        self.H=json.loads(self.seg_config["Homography_matrix"])
        self.results=self.config_object["Results_path"]
        self.save_results=self.results["Path"]
    

    def find_midpoint(self,pt_ls):
        '''
        Find the midpoint(image coordinates) given 4 corners of the contour
        '''
        m1,c1 = ret_line_eq(pt_ls[0],pt_ls[2])
        m2,c2 = ret_line_eq(pt_ls[1],pt_ls[3])

        #Solve the 2 eqns to obtain the midpoint
        A = np.array([[-m1,1],
                    [-m2,1]],dtype=np.float64)
        B = np.array([c1,c2])
        midpoint = np.linalg.inv(A).dot(B)
        return midpoint 
   
    def world_2d(self,points_ls):
        '''
        Convert a list of 3d points to corresponding image points using homography
        '''

        H=np.array(self.H)
        points_2d_ls = []
        for point in points_ls:
            point_3d = np.asarray(point)
            point_3d = point_3d.reshape(3,1)
            
            point_2d = np.dot(H,point_3d)
            point_2d = point_2d // point_2d[2,0]
            points_2d_ls.append(point_2d)
            
        return points_2d_ls

    def run_seg(self,frame,world_points):
        '''
        Function to run segmentation using  image
        '''
        #world points are the 3d points  
        self.points_2d_ls=self.world_2d(world_points)
        #prepare the image to pass to shelfnet network
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_pil = Image.fromarray(frame)
        #get the segmented image

        self.seg_img,conf= self.auto_park_obj.forward_pass(frame_pil,img_path=None)

        if conf<0.6:
            return [],self.seg_img

        #get the parking spot i.e the road region bounded by the roi,indicated by 
        #self.rect_pts
        res_img,self.rect_pts = pot_parking_spot(frame,self.seg_img,self.points_2d_ls,self.save_results)

        
        #Error handling
        for i in range(0,len(self.rect_pts)):
            if(len(self.rect_pts[i]) == 0):
                return [],res_img #Failure
            if int(self.rect_pts[0][1])==int(self.rect_pts[1][1]) or int(self.rect_pts[2][1])==int(self.rect_pts[3][1]):
                return [],res_img
        
        return self.rect_pts,res_img


if __name__ == '__main__':
    # img = cv2.imread('/content/homography_computation/data/rbc_data_00061.png')
    carla_utils_obj = carla_utils(intrinsic_mat)
    
    #debug
    # for img_path in glob.glob('/media/smart/5f69d8cc-649e-4c33-a212-c8bc4f16fc67/CARLA_0.8.4/PythonClient/Course1FinalProject/_out/*.png'):
 
    #     frame = cv2.imread(img_path)
    #     midpoint = carla_utils_obj.run_seg (frame,ext_mat,[229,129,38])
    #     print("midpoint",midpoint)
    