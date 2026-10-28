# 新云台方向与限位记录阶段

最新确认：原始IMU Roll允许-35°～+5°，抬头使Roll减小，反方向增大。
PitchSpeedTune的目标钳位和实际位置方向限位均使用该范围；下文旧范围均已被本项覆盖。

## 当前阶段：Pitch独立速度环

当前新双环已接遥控器：复用remote_init、rc_arbitration、RC_Levers和portGetCh1。
先发送kpp=非零值进入位置模式，右摇杆上下以速度命令累加位置目标；回中保持。
满杆目标变化速度为PitchSpeedTune::RemoteMaxSpeedDeg=30°/s，目标Roll钳制到-5°～35°。
掉线沿用旧online/live_time判定，失效后摇杆量为0，不再累加，仍保持目标；不新增电机停机逻辑。
单速度模式、关闭状态不响应摇杆位置输入，stop后摇杆不自动重新使能。
DR16使用USART3 RX=PC11，原100000波特、8数据位+偶校验、1停止位；右拨杆上/中为摇杆模式。
VT03沿用USART6 RX=PG9，mode_sw非0为摇杆模式。Yaw尚未接入新闭环，始终零输出。

新增显式模式切换：`position_enable`捕获当前原始Roll作为保持目标，执行位置/速度串级PID。
发送`kpp=0.4`在线设置位置环Kp，并自动启用位置环，无需另发position_enable；
首次切换捕获当前位置，已在位置模式时只改Kp、不重设目标。显式停止/模式命令优先。
`p:2`使目标抬头2度，即目标原始Roll减少2度；目标钳制到Roll -5°～35°。
`speed_enable`或原`pitch_enable`切回目标速度0的单速度环；`stop`关闭两种模式。
重复position_enable不重设已有目标。位置反馈=-Roll，速度反馈=-gy，正电流抬头。
新双环仍复用CAN_Demo PID及counts/rpm尺度，不接入旧TY角度环或自动回位。
CAN发送层电流限幅跟随PitchImuHold::CurrentLimit，不再固定在旧2000。

最新：用户已接回电机，PitchSpeedDryRun=false。恢复Pitch ID3实际电流发送、
电机反馈有效条件和原始Roll -5°～35°限位，Yaw ID1保持零输出。
上电保持关闭，pitch_enable启用，stop停止；空板演算段落为上一阶段记录。

当前用户仅连接C板：PitchSpeedDryRun=true。速度环仅要求有效IMU，不要求电机反馈，
不提交CAN电流帧、不执行机械角度限位；IP_cmd表示计算值，IP_fb固定0。
上电不使能，pitch_enable启动、stop停止，kp=/ki=/kd=在线修改速度环参数。
接回电机时将PitchSpeedDryRun改为false，才恢复电流发送及原始Roll -5°～35°限位。

用户更换C板后确认串口收发恢复，SerialTestOnly=false，恢复速度环调试及每20ms精简打印。

`PitchSpeedTuneMode=true`，`CommissioningMode=false`；用户已关闭持续电流测试，
持续电流值为0。上电发送零指令，`pitch_enable`启动目标速度为0的速度PID，
`stop`/`current=0`/`pitch_disable`停止；不执行角度环、自动回位、遥控或视觉。
`kp=`/`ki=`/`kd=`分别在线修改此速度PID，默认值在PitchImuHold.hpp调参区，
运行实例为Gimbal.Pitch_Speed_Tune.pid.SpdParam。`kpp=`在本阶段不应用。
速度反馈为-gy（rad/s转等效rpm），正电流抬头，因此PID出口不取负。
用户最新指定控制限位为原始IMU Roll=-5°～35°，不是取反后的Pitch；到边界或范围外只限制向外电流，
不因越界关闭速度环。速度死区为0，不再忽略12°/s以下运动。
Yaw ID1始终为0，Pitch ID3输出速度PID电流。
当前打印只保留R_deg、gx_dps、gy_dps、gz_dps、IP_cmd、IP_fb、IMU，
IP_cmd是实际提交的控制输出，IP_fb为电机反馈；R_deg后保留双逗号。
以下持续电流及测量内容为历史阶段说明，不代表当前自动输出。

## 当前覆盖设置：仅Pitch持续正电流

用户要求在RTOS每2ms持续发送Yaw ID1=0、Pitch ID3=+3000原始电流指令。
`GimbalCommissioning::ContinuousCurrentTest=true`，绕过以下旧500ms脉冲流程，
不运行PID和旧限位。上电后自动输出。`stop`或`current=0`锁存两轴零输出，
直到重新上电；CAN发送异常当次仍尝试提交零，下一周期继续用户指定输出。
下文零输出启动及脉冲说明仅在ContinuousCurrentTest=false时适用。

当前 `GimbalHardware::CommissioningMode=true`。上电周期发送两轴零电流，
不运行自动回位、PID、遥控器目标更新或视觉驱动。原PID参数保留。

USART1，115200，8N1；ASCII命令，每条追加CRLF：

| 命令 | 动作 |
|---|---|
| `pitch_plus` | Pitch ID3 +600，500ms |
| `pitch_minus` | Pitch ID3 -600，500ms |
| `yaw_plus` | Yaw ID1 +600，500ms |
| `yaw_minus` | Yaw ID1 -600，500ms |
| `stop` 或 `current=0` | 两轴立即提交零指令 |

每次只有一个轴非零，新方向命令会替换当前脉冲。原 `current=600`、
`current=2000` 保留为Pitch正向脉冲。此阶段不依赖旧IMU映射/限位，
不执行旧角度、速度、反馈时效停机条件；脉冲500ms在任务正常调度时结束。
零电流不制动，Pitch配重不平衡时需要手扶。

当前按用户要求，每20ms输出带标签文本：
`R_deg=...,P_deg=...,Y_deg=...,,gx_dps=...,gy_dps=...,gz_dps=...,EY=...,EP=...,IY_cmd=...,IP_cmd=...,IY_fb=...,IP_fb=...,IMU=...`。
R/P/Y是原解算欧拉角名称（P/R命名沿用旧工程）；gx/gy/gz单位°/s。
EY/EP是Yaw/Pitch编码器；IY/IP_cmd是原始电流指令，IY/IP_fb是电机反馈。
IMU表示姿态发布有效。保留原始读数，不在测量阶段改写它们的符号。
Yaw电机暂沿用ID1，Pitch按用户确认改为ID3；Pitch反馈为0x207，电流指令在第3路。

记录顺序：
1. 手动固定Yaw，抬头/低头：记录欧拉角/TY、三轴角速度和EP增减。
2. 固定Pitch，Yaw正反转：记录角度、角速度和EY；换俯仰姿态重复。
3. 手扶机构逐轴发送正负脉冲，记录正电流的真实转向。
4. 枪口水平处记录IMU和EP；机械允许上下边界处记录IMU和EP。
5. Yaw若存在机械限位，记录左右边界；否则明确记录无限位及线缆约束。

| 项目 | 实测结果 |
|---|---|
| C板安装方向、枪口前向 | 待记录 |
| 抬头时IMU角、角速度和EP变化 | 用户确认R从约0减小至-20度；角速度/EP方向待记录。抬头正候选反馈=-R |
| Yaw正转时IMU角、角速度和EY变化 | 用户确认左转Y增加；角速度/EY方向待记录。左转正候选反馈=Y |
| Pitch正电流方向 | 用户确认抬头，IMU Roll减小；抬头正角反馈=-Roll，控制电流出口取正（与旧云台出口取负不同） |
| Yaw正电流方向 | 用户确认向左，IMU Yaw增加 |
| 水平零位 | 用户确认IMU Roll=0°同时是枪口水平位置和Pitch机械边界 |
| Pitch上下边界 | 用户确认原始IMU Roll为-28°～0°；按抬头正Pitch=-Roll，对应0°～28°。Roll=0是范围端点，是否水平零位尚未单独确认 |
| Yaw左右边界/线缆范围 | 用户确认Yaw无限位；后续不配置Yaw机械角度限位 |

记录完成后再配置机械轴映射、零位和限位，并退出测量模式调PID。
