# msg/VisionResult.msg - 视觉检测结果消息
std_msgs/Header header
int32 code                              # 结果代码 (>1:检测数量, 0:空箱, -1:有点云无坐标, -2:非法指令, -3:未开启检测)
string error_message                    # 错误信息
geometry_msgs/Pose[] poses              # 检测到的位姿列表

