# YoloInfo.msg
std_msgs/Header header

float32 confidence # 0-1 (ex. 0.95)
uint16 frame_id
bool detected
# pose.position.x/y are image pixel coordinates
# pose.position.z is unused
geometry_msgs/Pose pose
