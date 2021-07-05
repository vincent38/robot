export ROBAIR_ARDUINO=/dev/serial/by-id/usb-Arduino_Srl_Arduino_Mega_5563530373835180A2E0-if00
export MONITOR_PORT="$ROBAIR_ARDUINO"


cd /home/robair/RobAIR/catkin_ws
catkin_make install

cd /home/robair/RobAIR/arduino/libraries

rm -rf ./ros_lib
echo "hi"
rosrun rosserial_arduino make_libraries.py /home/robair/RobAIR/arduino/libraries
echo "bye"

cd /home/robair/RobAIR/arduino/robairarduino

make clean && make upload
