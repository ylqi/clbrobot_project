rm -rf /home/clbrobot/catkin_ws/src/clbrobot_project/clbrobot/maps/house.pgm 
rm -rf /home/clbrobot/catkin_ws/src/clbrobot_project/clbrobot/maps/house.yaml

rosservice call /write_state /home/clbrobot/catkin_ws/src/clbrobot_project/clbrobot/maps/map.bag.pbstream

rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=/home/clbrobot/catkin_ws/src/clbrobot_project/clbrobot/maps/map.bag.pbstream


cp  /home/clbrobot/catkin_ws/src/clbrobot_project/clbrobot/maps/map.pgm /home/clbrobot/catkin_ws/src/clbrobot_project/clbrobot/maps/house.pgm

cp  /home/clbrobot/catkin_ws/src/clbrobot_project/clbrobot/maps/map.yaml /home/clbrobot/catkin_ws/src/clbrobot_project/clbrobot/maps/house.yaml

