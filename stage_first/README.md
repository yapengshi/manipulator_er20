# Ros-gazebo simulation for manipulator ER20-1700

# Notice
1. mH110.txt 		刘部长发来的机器人关节角度　units/degree
2. RawAngles.txt 	根据mH110.txt导出的程序可读文件，请勿删除；

3. InterPolation.txt	根据RawAngles进行10倍插值，写入存储文件，每次运行均更新
4. CalibratedAngles.txt	根据误差校正后获得的新关节角度（可与InterPolation.txt对比查看关节校正度数），写入存储文件，每次运行均更新


# Use
1. 启动gazebo仿真 roslaunch ros_robotics rrbot_gazebo.launch
2. 启动跟随程序   rosrun stage_first main
3. 用户自定义依次输入ｘ,y,theta偏差值(注意为较小偏差)：Enter the offset x/y/theta:	/单位：m/m/rad。
4. 测试代码是否正确：
a) Original XYZ=　xxx	k时刻引入校正前末端位置；
b) Calibrated XYZ= xxx　	k时刻引入校正后末端位置；
c)　Offset Verify =　xxx	k时刻校正前后末端位置偏差（与误差输入值对比）
----
视频中输入偏差（0.1 0.08）和角度（可以直接变工件角度，不影响机械臂）
根据原始关节角度和引入校正后的关节角度，正运动学计算得到末端位置 Original XYZ= ,Calibrated XYZ=
相减后偏差为（0.1,   0.0799998），对比输入 0.1 0.08，说明程序没问题。
----



