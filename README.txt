The aim of this project is to implement a gesture recognition system using Kinect camera, ROS and PCL as tools.
The initial goal was to recognize at least three gestures in order to play the popular game Rock Paper Scissor
against the computer, but with the current implementation it is trivial to add support for X more gestures.
The recognition rate achieved depends logically on the training files, but if these are good, it should be around 0.8

Our recognition implementation is heavily based on the PCL tutorial on cluster recognition and 
viewpoint feature histogram (VFH) descriptors by Radu B. Rusu, which can be found under the following link 
http://pointclouds.org/documentation/tutorials/vfh_recognition.php#vfh-recognition

We also rely on Garratt Gallagher's hand detection ros package for the segmenation. See the following link
for installation http://www.ros.org/wiki/mit-ros-pkg/KinectDemos/HandDetection

Used tools:
  Hardware
    Computer
    Microsoft Kinect Camera
  Software 
    Ubuntu 11.04
    ROS Diamondback
    PCL 1.3.1
    Hand Detection stack
    Openni stack
    Opencv2 stack
    
How to compile and run the application
--------------------------------------
First of all install ROS, PCL, openni and the hand detection stack(see above).
give the starting script executable permissions with 
$ chmod u+x launcher_script.sh
then run the script
$ launcher_script.sh
a new terminal with two tabs should be opened. In the first tab, openni and the hand detector node should be running
and in the second tab, our main program. There you can choose between learning mode, playing mode and bootstrap mode.
Learning mode serves to capture and label new training files and in playing mode you just play against a randomizer. 
Bootstrap on the other hand, serves to capture a lot of gestures which are all labeled equally.
Note that our program subscribes to /hand1_fullcloud topic, so until
the hand detector node tracks your skelet (and starts publishing in that topic), our program will not start

Comments
--------
Beware of the bootstrapping option, it is easy to capture transition gestures that worsens the quality of the training files.
Start only bootstrap mode if you capture the right gesture from start to end.



Known Issues
------------
If you remove training files, the files training_data.h5, training_data.list and kdtree.idx are not reloaded until a new gesture has been saved.


