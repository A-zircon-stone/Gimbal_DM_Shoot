# 用原视觉协议发送一次模拟Pitch目标

## 本次接入的范围

复用`VisionReceivePacket2026`的31字节布局、SP帧头和原CRC16，仍走C板USB CDC。
只把`pitch`接入新Pitch IMU闭环；Yaw、角速度/加速度前馈及开火暂不响应。
`pitch`在这个新接口中是绝对IMU角度rad，抬头正，对应精简打印第0列；不是水平机械零点，也不是角度误差。
旧自瞄分支的Pitch负号约定未套入新接口。视觉上位机正式接入时必须采用此约定或显式做转换。

## 使用

1. 烧录本工程新版固件，Pitch放在行程中间，支撑机构，准备断电。
2. 另外用数据线连接C板自身USB到电脑。DAPLink的COM5仅是当前USART1桥接，不是USB视觉口。
3. VOFA保持连接USART1，追加CRLF发送`vision_enable`。它启用闭环并只开放随后新收到的视觉帧，不消费之前缓存的目标。
4. 双击`tools/模拟视觉抬头2度.cmd`。脚本自动找VID0483/PID5740的C板USB CDC，不能同时被其他程序占用。
5. 按提示回车。脚本从USB读取最新`pitch:`实际角度，计算当前角度+2度，然后只写一次31字节二进制包，不循环发送。
6. USART1日志的目标角度应增加约2度。发送`vision_disable`/`pitch_disable`/`pitch_stop`/`current=0`停止；这些命令仍走USART1。

脚本无需安装Python或pyserial。CRC/包长度可用`powershell.exe -File tools/send_vision_pitch_once.ps1 -SelfTest`验证，此参数不会打开串口。
模拟脚本只允许预计+2度后仍处在3550～6350软范围的起点；不会修改控制器限位行为。
当前板端到达限位依然发零并锁定退出，不是将任意越界目标钳到边界后持续保持。

## 模式与优先级

上电默认不接受视觉目标驱动电机，须先经USART1显式`vision_enable`。
之后mode=1的新CRC有效包可以更新Pitch目标；mode=0使Pitch发零停止。
手动命令会退出视觉控制，后到视觉帧不能自行重启。
不增加丢包/断线超时动作：一条mode=1指令后，目标持续保留，直到手动停止/新指令或既有限位、非法数据、CAN故障。
完整USB接收仍复用旧解析器：本次单写31字节适用；任意拆包/粘包尚未做通用流解析改造，正式视觉链路需另行验证。

## 帧布局

0..1=SP；2=mode；3=fire；4/8/12=yaw/yaw_vel/yaw_acc；16/20/24=pitch/pitch_vel/pitch_acc；28=shoot_count；29..30=CRC低/高字节。
float为32位小端。CRC初值0xFFFF，反射多项式0x8408，无最终异或，对前29字节计算。
固定pitch=-1rad、mode1、其余数值0的样本CRC为D5 1D，已与固件原CRC函数、Python标准库构包和PowerShell脚本交叉验证。
只有CRC校验通过才发布接收序号；GIMBAL任务从短临界区捕获完整包，再在任务中更新PID目标，不在USB中断里驱动电机。

## 验证边界

固件编译及主机回归通过，不等于已完成USB/电机板测。脚本未自动运行发送，固件未自动烧录。
