#uarm臂
cd lerobot

#标定舵机主臂零位（静止挂臂 + 按下扳机采行程）
lerobot-calibrate \
    --teleop.type=bi_openarm_servo \
    --teleop.left_arm_config.port=/dev/ttyUSB1 \
    --teleop.right_arm_config.port=/dev/ttyUSB0 \
    --teleop.id=my_openarm_servo

#设置从臂can接口
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
sudo ip link set can1 up

#标定从臂零位
lerobot-calibrate \
    --robot.type=openarm_follower \
    --robot.port=can0 \
    --robot.side=right \
    --robot.id=my_openarm_follower_right

lerobot-calibrate \
    --robot.type=openarm_follower \
    --robot.port=can1 \
    --robot.side=left \
    --robot.id=my_openarm_follower_left

#舵机主臂遥操作从臂（串行读 8 路舵机，fps 建议 20–30）
lerobot-teleoperate \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can1 \
    --robot.left_arm_config.side=left \
    --robot.right_arm_config.port=can0 \
    --robot.right_arm_config.side=right \
    --robot.id=my_openarm_follower \
    --teleop.type=bi_openarm_servo \
    --teleop.left_arm_config.port=/dev/ttyUSB1 \
    --teleop.right_arm_config.port=/dev/ttyUSB0 \
    --teleop.id=my_openarm_servo \
    --fps=30 \
    --display_data=false


#mini臂
cd lerobot

#标定mini主臂零位
lerobot-calibrate \
    --teleop.type=bi_openarm_mini \
    --teleop.left_arm_config.port=/dev/ttyACM1 \
    --teleop.right_arm_config.port=/dev/ttyACM0 \
    --teleop.id=my_openarm_mini

#设置从臂can接口
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
sudo ip link set can1 up

#标定从臂零位
lerobot-calibrate \
    --robot.type=openarm_follower \
    --robot.port=can0 \
    --robot.side=right \
    --robot.id=my_openarm_follower_right

lerobot-calibrate \
    --robot.type=openarm_follower \
    --robot.port=can1 \
    --robot.side=left \
    --robot.id=my_openarm_follower_left

lerobot-teleoperate \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can1 \
    --robot.left_arm_config.side=left \
    --robot.right_arm_config.port=can0 \
    --robot.right_arm_config.side=right \
    --robot.id=my_openarm_follower \
    --teleop.type=bi_openarm_mini \
    --teleop.left_arm_config.port=/dev/ttyACM1 \
    --teleop.right_arm_config.port=/dev/ttyACM0 \
    --teleop.id=my_openarm_mini \
    --fps=60 \
    --display_data=false

