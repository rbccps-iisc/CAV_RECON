
This is the vision code pipeline,works as follows-

1.Capture the frame
2.Run object detection
3.Get the distance of the object and alert the driver if necessary
4.Run road segmentation (if required by the driver,set it in config file),and using homography suggest a goal point at the side of the road for parking,provided no obstacle is in the region around the goal point.
The entire pipeline(comprising of object detection,lidar based distance estimation and segmentation-based goal point generation) runs at 10fps on a gtx 1060 laptop for a VGA image input.The pipeline can be configured,using the config.ini file.

Usage 
case 1- If ROS=0 in config.ini
python3 homo_objnew_lidar_live.py

Dependencies-
Torch>1.2.0
cuda=10.2
cudnn=7.6
python-opencv-4.5
numpy
numba

Case2-if ROS=1 in config.ini

These steps are valid for a python3-virtual environment.
1.Install ros-melodic from the official wiki page.(no need to build catkin_ws)
2.sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy
3.Follow the below instructions in virtual environment.These are only done so that the code works with python3-ros
mkdir -p ~/catkin_ws/src; cd ~/catkin_ws
catkin_make
source devel/setup.bash
wstool init
wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5
wstool up
rosdep install --from-paths src --ignore-src -y -r
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
source ./devel/setup.bash
pip3 install netifaces

followed all instructions from this link https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/

4.Once ros is installed and working for python3,open another terminal and run roscore and run python3 homo_objnew_lidar_live.py in another terminal.
5.In another terminal play the bag file or if lidar and camera is connected to the laptop,launch the files to get input data from the sensors.

Sample bag file- https://drive.google.com/drive/folders/1_P3v1fr4pMGGdf5ctAGZwCAQfsZq8MYP?usp=sharing

The weights and cfg file for object detection can be used from https://drive.google.com/drive/folders/1oYDBL6aKqFbLG9WHZtr-9p3tba1K-TVI?usp=sharing

Segmentation weights - https://drive.google.com/file/d/1dCIFkrJYRv_diVD3q7uxazlu-CJv1tif/view?usp=sharing

Sample video link -https://drive.google.com/file/d/1ldeuFCHQ1VVGGs0AzYdygu_Gv_cDvd7a/view?usp=sharing

Repositories referred-

1.Darknet- alexyAB - https://github.com/AlexeyAB/darknet (version- 64efa721ede91cd8ccc18257f98eeba43b73a6af )
2.Shelfnet - https://github.com/juntang-zhuang/ShelfNet
3.Bisenet - https://github.com/CoinCheung/BiSeNet

(the virtual environment used on gtx 1060 is onx (source ~/deployment/onx/bin/activate)

TODO:
1.Improve accuracy of object-detection,initially by collecting more data from the campus (car,bicycle,animal)
2.Try to track the goal point using odometry based method.
3.Do tests around the campus for this pipeline.
4.To try a more robust alert system for the driver,by tracking objects.
5.Extend the entire pipeline to multiple cameras in real time.
