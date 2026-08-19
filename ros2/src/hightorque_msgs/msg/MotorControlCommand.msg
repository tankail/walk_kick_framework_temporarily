# 电机控制命令消息
string uuid                    # 控制会话UUID
int32[] motor_ids              # 本次控制的电机ID
float32[] positions            # 目标位置（rad）
float32[] velocities           # 目标速度（rad/s）
float32[] kp                   # 位置增益（可选）
float32[] kd                   # 速度增益（可选）
float32[] torques              # 前馈力矩（可选）
