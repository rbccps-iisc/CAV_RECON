
**This is the vision code pipeline,works as follows-**

1.Capture the frame <br />
2.Run object detection <br />
3.Get the distance of the object and alert the driver if necessary <br />
4.Run road segmentation (if required by the driver,set it in config file),and using homography suggest a goal point at the side of the road for parking,provided no obstacle is in the region around the goal point. <br />

The entire pipeline(comprising of object detection,lidar based distance estimation and segmentation-based goal point generation) **runs at 10fps on a gtx 1060** laptop for a VGA image input.The pipeline can be configured,using the config.ini file. <br />

**Usage**

**case 1- If ROS=0 in config.ini** <br />
python3 homo_objnew_lidar_live.py <br />

**Dependencies-** <br />
Torch>1.2.0 <br />
cuda=10.2 <br />
cudnn=7.6 <br />
python-opencv-4.5 <br />
numpy <br />
numba <br />

**Case2-if ROS=1 in config.ini**

These steps are valid for a python3-virtual environment. <br />
1.Install ros-melodic from the official wiki page.(no need to build catkin_ws) <br />
2.sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy <br />
3.Follow the below instructions in virtual environment.These are only done so that the code works with python3-ros <br />

mkdir -p ~/catkin_ws/src; cd ~/catkin_ws <br />
catkin_make <br />
source devel/setup.bash <br />
wstool init <br />
wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5 <br />
wstool up <br />
rosdep install --from-paths src --ignore-src -y -r <br />
catkin_make --cmake-args 
            -DCMAKE_BUILD_TYPE=Release  
            -DPYTHON_EXECUTABLE=/usr/bin/python3   
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m 
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so <br />
source ./devel/setup.bash <br />
pip3 install netifaces <br />

followed all instructions from this link https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/

4.Once ros is installed and working for python3,open another terminal and run roscore and run python3 homo_objnew_lidar_live.py in another terminal. <br />
5.In another terminal play the bag file or if lidar and camera is connected to the laptop,launch the files to get input data from the sensors.<br />

**Sample bag file**- https://drive.google.com/drive/folders/1_P3v1fr4pMGGdf5ctAGZwCAQfsZq8MYP?usp=sharing

**The weights and cfg file for object detection can be used from** https://drive.google.com/drive/folders/1oYDBL6aKqFbLG9WHZtr-9p3tba1K-TVI?usp=sharing

**Segmentation weights** - https://drive.google.com/file/d/1dCIFkrJYRv_diVD3q7uxazlu-CJv1tif/view?usp=sharing

**Sample video link** -https://drive.google.com/file/d/1ldeuFCHQ1VVGGs0AzYdygu_Gv_cDvd7a/view?usp=sharing

**Repositories referred-** <br />

1.Darknet- alexyAB - https://github.com/AlexeyAB/darknet (version- 64efa721ede91cd8ccc18257f98eeba43b73a6af ) <br />
2.Shelfnet - https://github.com/juntang-zhuang/ShelfNet <br />
3.Bisenet - https://github.com/CoinCheung/BiSeNet <br />

(the virtual environment used on gtx 1060 is onx (source ~/deployment/onx/bin/activate) <br />

**TODO:** <br />
1.Improve accuracy of object-detection,initially by collecting more data from the campus (car,bicycle,animal) <br />
2.Try to track the goal point using odometry based method. <br />
3.Do tests around the campus for this pipeline. <br />
4.To try a more robust alert system for the driver,by tracking objects. <br />
5.Extend the entire pipeline to multiple cameras in real time.<br />
