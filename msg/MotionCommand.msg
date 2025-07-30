# 运动命令消息定义
std_msgs/Header header

# 机械臂标识
string robot_id

# 运动类型 (JOINT_MOVE=0, CARTESIAN_MOVE=1, LINEAR_MOVE=2, CIRCULAR_MOVE=3)
int32 motion_type

# 关节位置 (用于关节运动)
float64[] joint_positions

# 目标位姿 (用于笛卡尔运动)
geometry_msgs/Pose target_pose

# 运动参数
float64 speed_ratio         # 速度比例 (0.0-1.0)
float64 acceleration_ratio  # 加速度比例 (0.0-1.0)
bool wait_for_completion   # 是否等待运动完成

# 附加参数
string motion_params       # JSON格式的附加参数
