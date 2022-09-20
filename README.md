GA learning:

Pick and place:

roslaunch kinect2_bridge kinect2_bridge.launch publish_tf:=true depth_registration:=true

roslaunch moveit_check baxter_base_trans.launch

roslaunch ar_track_alvar pr2_indiv.launch

rosrun baxter_interface joint_trajectory_action_server.py

roslaunch baxter_moveit_config baxter_grippers.launch

roslaunch moveit_check object_position_server.launch

rosrun moveit_chectest_moveit_table_top_assemble_scenario.py

rosrun sound_play soundplay_node.py


(you can find it in the moveit_check repo)


Running archi:

rosrun launching_file_from_web hierarchicalGA.py

rosrun launching_file_from_web launching_file_fromweb_2

rosrun launching_file_from_web launching_file_fromweb_4

Ros js page:

roslaunch rosbridge_server rosbridge_websocket.launch

python3 -m http.server 8090


