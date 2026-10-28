# VOFA+ 1.3.10 立方体

## 为什么R/P/Y不能直接绑定

接收区中的`t=... R=... P=... Y=...`只是普通文字，不是已解析的数值通道。
控件右键菜单列出的是右侧“数据”栏中的通道，不会从日志变量名自动建立通道。
将旧通道改名R只会改名字，不能改变它实际收到的数据内容。

## 本版本连接

工程：E:\STM32Projects\2026_Balanced_Gimbal_6020。
重新构建并下载GM6020_Feedback，连续运行。固件仍不发送电机命令。
USART1与USB CDC均每100ms输出一行FireWater CSV，不混入标签日志。
USART1保持115200、8N1，原接线不变。USB停止旧VISION二进制发送；不通过滑块/按钮控制电机。

1. VOFA+选择USART1的USB-TTL适配器COM口（或C板自身USB CDC），协议引擎选择FireWater。
2. 接收区应出现`att:...,...,...`数字行，而不是旧的`t=... R=...`。
3. 按列顺序把前4个数据通道重命名为q0、q1、q2、q3。
   VOFA+可能保留旧工程通道名（例如yawtarget），不要按旧名猜数据。
4. 立方体右键切换到四元数模式：scalar绑定q0，X绑定q1，Y绑定q2，Z绑定q3。
5. 不需要选择角度/弧度，四元数无此单位。世界视角/模型朝向可以用“模型视图”或“位姿偏置”调整。
6. 波形时间轴Δt设100ms，符合实际10Hz发送周期；改Δt不改变发送频率。

当前只是传感器姿态可视化，不是“上方Pitch/下方Yaw”的两自由度机械模型。
初始Yaw参考、传感器安装方向与模型朝向不同不代表控制轴已标定。
模型显示效果尚需上位机实测，不以编译/解析测试代替。

## 固定通道顺序（从0开始）

|列号|内容|单位|
|---|---|---|
|0..3|q0(w),q1(x),q2(y),q3(z)|无|
|4,5,6|INS.Roll(R),INS.Pitch(P),INS.Yaw(Y)|度|
|7,8,9|gx,gy,gz|rad/s|
|10,11|TY,TV|度、有效标记|
|12,13|EY,EP|编码器原值|
|14,15|rpmY,rpmP|rpm|
|16,17|OY,OP|在线标记|
|18|IMU|有效标记|
|19,20|t,N|ms、计数|
|21..23|EKF额外偏置XYZ|rad/s|
|24..26|EKF实际预测角速度XYZ|rad/s|
|27,28|温度、dt|℃、s|
|29..31|加速度XYZ|m/s²|
|32,33|内部四元数范数、加速度模长|无、m/s²|

当前只在IMU新鲜、四元数和其他显示输入有效时发送采样行，IMU停更时USB数据可能冻结；
如需人工标签日志，将PrintTelemetry.hpp的PrintUart1FireWater改为false后重新构建下载，
此时USART1不再适合VOFA绑定；不要在同一通道混发。四元数仅在输出副本归一化，不修改INS状态。

## 如坚持使用欧拉模式

老工程命名不同于常见Roll-X/Pitch-Y：QEKF_INS.Pitch是绕X的atan2结果，
QEKF_INS.Roll是绕Y的asin结果。不能直接按X=R、Y=P照搬。
可先尝试X绑定第5列(P)、Y绑定第4列(R)、Z绑定第6列(Y)，单位角度；
但VOFA+控件的组合旋转约定和模型坐标仍需实测，优先使用原始四元数。
TY不是欧拉Y角，不要拿TY替换四元数中的Y分量。

官方依据：
- https://www.vofa.plus/docs/learning/dataengines/firewater/
- https://www.vofa.plus/docs/learning/widgets/cube/
- https://www.vofa.plus/docs/learning/start/data_cmd_parameter/
