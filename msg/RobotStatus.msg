# 机械臂状态消息定义
std_msgs/Header header

# 机械臂标识
string robot_id

# 机械臂型号
string robot_model

# 连接状态
bool is_connected

# 运动状态 (DISCONNECTED=0, CONNECTING=1, IDLE=2, MOVING=3, ERROR=4, EMERGENCY_STOP=5)
int32 status

# 当前关节角度
float64[] current_joints

# 当前位姿
geometry_msgs/Pose current_pose

# 当前速度
float64 current_speed

# 错误信息
string error_message
int32 error_code

# 负载信息
float64 current_payload

# 系统信息
float64 cpu_usage
float64 memory_usage
string firmware_version
