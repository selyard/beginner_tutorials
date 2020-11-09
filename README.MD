[MIT License]

Instructions to launch code:

(0) Code generated using ROS Melodic

(1) Navigate to: <catkin workspace>/src/

(2) Run: git clone https://github.com/selyard/beginner_tutorials.git

(3) Navigate to <catkin workspace> and run: catkin_make

(3) Run in this terminal: roslaunch beginner_tutorials begin_tut_launcher.launch

(3a) Optional: Define a frequency of output messages using the argument freq:=(integer) ; defaults to 10 (hz).


Instructions to test code:

(1) Navigate to <catkin workspace> and run: catkin_make test

(2) Run in this terminal: roslaunch beginner_tutorials test_talker_launch



To call new service:

(0) Launch code as noted above

(1) In new console, type: rosservie call make_string_better <input string>

(2) Console will output new string as part of an output string.




CPPCheck output included in: "output_cppcheck.txt"

CPPLint output included in: "output_cpplint.txt"

To run CPPCheck and CPPLint:

(1) Navigate to beginner_tutorials directory

(2) Run: chmod +x check_cppcheck.sh check_cpplint.sh

(3) To run CPPCheck: ./check_cppcheck.sh

(4) To run CPPLint: ./check_cpplint.sh

