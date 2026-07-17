#uarm
cd lerobot

#Calibrate the servo motor main arm to zero position (stationary arm + trigger press travel).
lerobot-calibrate \
    --teleop.type=bi_openarm_servo \
    --teleop.left_arm_config.port=/dev/ttyUSB1 \
    --teleop.right_arm_config.port=/dev/ttyUSB0 \
    --teleop.id=my_openarm_servo

#Configure the arm CAN interface
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
sudo ip link set can1 up

#Calibrate from arm zero position
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

#Servo master arm remotely controls slave arm (serial read 8 servo channels, recommended fps 20–30)
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


#mini arm
cd lerobot

#Calibrate the mini main arm to zero position
lerobot-calibrate \
    --teleop.type=bi_openarm_mini \
    --teleop.left_arm_config.port=/dev/ttyACM1 \
    --teleop.right_arm_config.port=/dev/ttyACM0 \
    --teleop.id=my_openarm_mini

#Configure the arm CAN interface
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
sudo ip link set can1 up

#Calibrate from arm zero position
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

