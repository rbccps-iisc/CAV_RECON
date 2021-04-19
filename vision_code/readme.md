**This is the vision code pipeline,works as follows-**

1.Capture the frame  
2.Run road segmentation  
3.Check if there is an object within the roi (using iou) given by the previous step.  
4.If no object in the roi , generate the 3d goal point using homography.  
5.If object is present brake immediately.  

It can also be configured(using config.ini) to run only one of the two models.A config.ini file has some parameters to be set for the pipeline to work.

**Usage** 

python3 homo_objnew.py

**Dependencies-**  
Torch>1.2.0  
cuda=10.2  
cudnn=7.6  
python-opencv-4.5  
numpy  

The code is tested on gtx 1060.The weights can be dwonloaded from this link-
Object detection - https://drive.google.com/file/d/1NJ2FfGzhOCx-IXJu9VyKz6a2MN3Kq04-/view?usp=sharing

Segmentation - https://drive.google.com/file/d/1dCIFkrJYRv_diVD3q7uxazlu-CJv1tif/view?usp=sharing

**Repositories referred-**

1.Darknet- alexyAB  
2.Shelfnet  
3.Bisenet  



