The main difference between the different folders is as listed below -  

**Version1** uses yolov4 darknet model for object detection and shelfnet as the segmentation model.It has the highest accuracy but **lowest fps- 9.5 on gtx 1060**.  

**Version2** uses yolov4_trt model and same shelfnet model with some modifications in code to get higher fps.This version has lower accuracy but higher **fps-19 on gtx 1060.**  

**Version1_lidar**- uses yolov4_tiny model for object detection.Shelfnet for segmentation,with some modifications same as that of version2 to get higher fps.This version also has lidar data included to get the distance of the detected objects.It can also use ROS live input to get lidar and camera data.Fps of this pipeline is  **10.3 on gtx 1060.**

**Each folder has a readme to set up the particular version**.

A docker container of version1 and version2 is uploaded - https://drive.google.com/file/d/10GQhyWgAsETzWwjTvhj2L-PcJzLru43p/view?usp=sharing  
Readme to use docker - https://drive.google.com/file/d/1JDOXcS9XxpJs1z-8iQ_rFvdCPHkyxgsk/view?usp=sharing  

The details of the models used in different versions is briefly summarized here- https://drive.google.com/file/d/1zFMCxbc7GfWz1ctPqgTKTd4Z7JXe3Vs1/view?usp=sharing

