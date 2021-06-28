export ROBAIR_ARDUINO=/dev/serial/by-id/usb-Arduino_Srl_Arduino_Mega_5563530373835180A2E0-if00
export MONITOR_PORT="$ROBAIR_ARDUINO"


cd /home/vincent/robair/RobAIR/catkin_ws
catkin_make install

rm -rf /home/vincent/robair/RobAIR/arduino/libraries/ros_lib
rosrun rosserial_arduino make_libraries.py /home/vincent/robair/RobAIR/arduino/libraries

cd /home/vincent/robair/RobAIR/arduino/robairarduino && make clean && make upload