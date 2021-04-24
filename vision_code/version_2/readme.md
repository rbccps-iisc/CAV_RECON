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
tqdm  
ninja  
TensorRT= 7.1.3  
torchvision=0.8.2  

The code is tested on gtx 1060.The weights can be dwonloaded from this link-  
Object detection - https://drive.google.com/file/d/18Yq550ggUbnDMg7O40lI1LvAYUsyUtep/view?usp=sharing  
Object detection tensorRT weights- https://drive.google.com/file/d/1ybtcJ5s-fD8862QE1hXfEin5klIXa3vT/view?usp=sharing  

Segmentation - https://drive.google.com/file/d/1RbF8TvPeNld1oURIQtKd1ORK6b1b9DIX/view?usp=sharing

Sample video link -https://drive.google.com/file/d/1ldeuFCHQ1VVGGs0AzYdygu_Gv_cDvd7a/view?usp=sharing

Note-1) The tensorRT model needs to built on the specific system on which it will be used.  

**Repositories referred-**

1.Darknet- alexyAB  - https://github.com/AlexeyAB/darknet  (version- 64efa721ede91cd8ccc18257f98eeba43b73a6af )  
2.Shelfnet  - https://github.com/juntang-zhuang/ShelfNet  
3.Bisenet  - https://github.com/CoinCheung/BiSeNet  
4.TKdnn-  https://github.com/ioir123ju/tkDNN




