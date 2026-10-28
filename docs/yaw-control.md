# 新云台Yaw闭环

Yaw为CAN1 GM6020 ID1，Pitch为ID3。左转IMU Yaw增加，正电流左转。
复用CAN_Demo的PID算法，位置误差按8192/360换成counts，速度rad/s换成rpm。
Yaw位置反馈使用INS.YawTotalAngle连续角，跨过±180°不跳变，无机械角度限位。
速度反馈由陀螺仪gy/gz及当前姿态换算为Yaw导数，兼顾Pitch倾斜时的投影。
INS旧命名中Roll是标准ZYX的Y俯仰，Pitch是标准X横滚。

USART1 115200 8N1，命令以CR/LF结束，每条单独发送：

- `ykp=35`：设置Yaw速度Kp，并直接进入目标速度0的速度环调试；不需要enable。
- `yki=0`、`ykd=0`：分别修改Yaw速度Ki/Kd，不切换模式。
- `ykpp=0.3`：设置Yaw位置Kp并进入串级位置环，首次切换捕获当前位置。
- `y:5`：在已有Yaw位置目标上向左增加5°；负数向右。仅位置模式有效。
- `ystop`：仅关闭Yaw；`stop`同时关闭两轴。
- 位置模式右摇杆水平控制Yaw目标角，居中保持目标；满杆30°/s，沿用原遥控负ch0方向。
- 再发`ykp=`会切回速度环；重复`ykpp=`保留已有目标，只修改位置Kp。
- 参数只在本次上电有效。编译默认值、9600电流限幅、位置环输出限幅在YawSpeedTune.hpp。
- Yaw上电首次取得合法IMU和电机反馈后，按代码中的PID参数自动捕获当前位置进入位置环；右摇杆水平直接控制。无需发送enable或PID命令。ystop/stop后不会自动重启；原ykp=/ykpp=模式切换命令保留。

Pitch参数保持PitchImuHold.hpp现有值，首次合法反馈时自动捕获当前位置进入位置环；
不自动回零。原Pitch调参命令及Roll -35°到+5°限位不变；stop后不会自动重启。
打印保留原字段，增加Y_deg/IY_cmd/IY_fb。电流字段为协议原始数值，不是安培。
