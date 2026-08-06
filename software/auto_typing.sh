# Align STEPS
# manual alignment to the Q key
# manual aligment needs to be slightly less than 5cm from the keyboard 

# ros2 service call move_perp_to_plane std_srvs/srv/Trigger 


# FIRST
# ros2 launch rover_arm_control rover_arm_auton.launch.py

# Lights red
# This spams topic I think
ros2 topic pub -t 3 /autonomous_LED/color rover2_status_interface/msg/LED "{red: 1, green: 0, blue: 0}"

# Send Goal
ros2 action send_goal auton_typing rover_arm_control_interface/action/AutonTyping {phrase: "d,a,m"}
 
# Lights Green
ros2 topic pub -t 3 /autonomous_LED/color rover2_status_interface/msg/LED "{red: 0, green: 1, blue: 0}"


