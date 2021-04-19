import torch
import torchvision.transforms as transforms
import cv2 
from PIL import Image
import numpy as np 
import glob
import sys
import time
import os
from configparser import ConfigParser

#scripts
config_object = ConfigParser()
config_object.read("config.ini")
path=config_object["Segmentation"]["path_for_segment_folders"]
sys.path.insert(1, path) # get the path from config file
from evaluate_for_conf import MscEval
from shelfnet import ShelfNet


class auto_park_vision():
    def __init__(self):

        self.n_classes = 19
        
        self.weights_path=path=config_object["Segmentation"]["weights_path"]
        if not os.path.exists(self.weights_path):
            print("Unable to find the segmentation weights,check the path again")
            sys.exit(0)
        #Point indicating potential parking spot
        self.midpoint = []
        self.point_3d = None #3D point corresponding to self.midpoint
        self.eval_define(self.weights_path) #Define Object of class MscEval
        #self.evaluator is an object of the class MscEval

    def forward_pass(self,frame=None,img_path=None):
        
        if(img_path != None):
            img = Image.open(img_path)
        else:
            img = frame 
        
        #Preprocess Image
        
        to_tensor = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
            ])
        img = to_tensor(img)


        _, H, W = img.shape
        
        #Change image size to the form NCHW from CHW
        img = img.unsqueeze(0)
        img = img.cuda()     

        prob = self.evaluator.scale_crop_eval(img, scale=1.0) #prob.type torch.cuda.FloatTensor
        prob = prob.detach().cpu()
        preds = prob.data.numpy()

        preds = preds.squeeze().astype(np.uint8)

        preds[preds == 0] = 255 #make the road region white,and send the preds as a image
        preds = preds.astype(np.uint8)

        return preds


    def image_to_world(self,point_2d):
        '''
        Function to find out world coordinates
        given an image point
        '''
        #TODO: Check homography matrix
       
        #Conversion to homogenous coordinates
        point_2d = np.append(point_2d,1).reshape(-1,1)

        H_inv = np.linalg.inv(self.H)
        self.point_3d = np.dot(H_inv,point_2d)
        self.point_3d = self.point_3d / self.point_3d[2,0]
        return self.point_3d




    def eval_define(self,weights_path):

        n_classes = self.n_classes
        net = ShelfNet(n_classes=n_classes)

        net.load_state_dict(torch.load(weights_path))
        net.cuda()
        net.eval()
        self.evaluator = MscEval(net, dataloader=None, scales=[1.0],flip=False)