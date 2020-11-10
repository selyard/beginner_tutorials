[MIT License]

Instructions to launch code:

(0) Code generated using ROS Melodic

(1) Navigate to: <catkin workspace>/src/

(2) Run: git clone https://github.com/selyard/beginner_tutorials.git

(3) Navigate to <catkin workspace> and run: catkin_make

(3) Run in this terminal: roslaunch beginner_tutorials begin_tut_launcher.launch

(3a) Optional: Define a frequency of output messages using the argument freq:=(integer) ; defaults to 10 (hz).

(3b) Optional: To record to a ROSBAG, use the argument rec:=true; defaults to false. Recording is saved in ~/.ros/ directory.



To inspect TF frame while code is running:

(1) rosrun tf tf_echo world talk

(2) rosrun rqt_tf_tree rqt_tf_tree



To call new service:

(0) Launch code as noted above

(1) In new console, type: rosservie call make_string_better <input string>

(2) Console will output new string as part of an output string.



Instructions to test code:

(1) Navigate to <catkin workspace> and run: catkin_make tests

(2) Run in this terminal: catkin_make test beginner_tutorials

Note: Image of test running successfully on my machine is included in Results/Week10/test_output.png



CPPCheck output included in: "output_cppcheck.txt"

CPPLint output included in: "output_cpplint.txt"

To run CPPCheck and CPPLint:

(1) Navigate to beginner_tutorials directory

(2) Run: chmod +x check_cppcheck.sh check_cpplint.sh

(3) To run CPPCheck: ./check_cppcheck.sh

(4) To run CPPLint: ./check_cpplint.sh


Deliverables: See Results/Week10 folder.
* frames.pdf: view_frames output
* rosbag_output.bag: Recording of talker for ~15s
* rosbag_playback.png: Result of playing back recorded file with talker listening
* Week10_RQTTree_RosEcho.png: Image showing rqt_tf_tree, rosecho, and running code.

