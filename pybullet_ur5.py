import pybullet as p
import time
import pybullet_data

# 启动模拟器
p.connect(p.GUI)  # 可以使用 p.DIRECT 以无图形模式运行
p.setGravity(0, 0, -9.81)

# 设置搜索路径以查找 URDF 文件
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 加载 UR5 机器人模型
ur5_id = p.loadURDF("/home/tong/pyb-sim-models/pbsm/models/UR5.urdf", basePosition=[0, 0, 0])

# 设置控制模式
p.setJointMotorControl2(ur5_id, jointIndex=0, controlMode=p.POSITION_CONTROL, targetPosition=0)
p.setJointMotorControl2(ur5_id, jointIndex=1, controlMode=p.POSITION_CONTROL, targetPosition=0)
p.setJointMotorControl2(ur5_id, jointIndex=2, controlMode=p.POSITION_CONTROL, targetPosition=0)
p.setJointMotorControl2(ur5_id, jointIndex=3, controlMode=p.POSITION_CONTROL, targetPosition=0)
p.setJointMotorControl2(ur5_id, jointIndex=4, controlMode=p.POSITION_CONTROL, targetPosition=0)
p.setJointMotorControl2(ur5_id, jointIndex=5, controlMode=p.POSITION_CONTROL, targetPosition=0)

# 运行模拟，并更新关节位置
for _ in range(24000):  # 运行 240 个时间步
    p.stepSimulation()
    time.sleep(1./240.)

# 断开连接
p.disconnect()
