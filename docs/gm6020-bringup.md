# 双 GM6020 适配：当前仅反馈验证

硬件约定：C板 CAN1（PD0 RX / PD1 TX），1 Mbps；下方 Yaw 电机ID 1，
上方 Pitch 电机ID 2。反馈分别为标准数据帧 0x205 / 0x206，DLC=8。
ID必须与电机拨码/实际指示一致。单圈编码器范围0..8191；速度反馈单位rpm。

依据：大疆 GM6020 使用说明 v1.2，第5–6页：
https://rm-static.djicdn.com/tem/17348/RoboMaster%20GM6020%E7%9B%B4%E6%B5%81%E6%97%A0%E5%88%B7%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E.pdf

## 当前边界

- GimbalHardware::FeedbackOnly=true：仅启动CAN1接收，不启动CAN2。
- 云台任务绕过旧遥控、自瞄、PID、MDO、摩擦轮与底盘命令链路。
- 工程三个CAN发送入口全部提前返回HAL_ERROR，不提交任何应用发送帧。
  普通CAN接收仍会在总线上应答ACK，并非硬件静默监听。
- 仅修改源码并构建，不自动下载、复位或使能电机。
- 这不是已完成的双轴闭环版本。旧电压/力矩接口、PID参数、IMU方向、
  限位、失联停止和恢复策略尚未适配，不能仅关闭FeedbackOnly就投入使用。
- 旧IMU任务保留；当前反馈验证任务不更新Gimbal.Yaw_Angle/Pitch_Angle，
  要查看旧姿态结果应看INS，电机位置看下述RawEcd/Angle。

## 下载后的验证

先支撑好云台，排除其他控制器或PWM信号使电机动作；终端电阻仅接在总线两端。
本版本不提供电机保持力，Pitch可能因重力下落，注意机械支撑。

监视Gimbal.Motor_Yaw和Gimbal.Motor_Pitch：

- ID：分别1/3；FeedbackCount：连续运行时应持续递增。
- FeedbackSeen：收到第一个有效帧后true；LastFeedbackMs：最后有效反馈时刻。
- 在线判定不缓存：消费方现场调用 gm6020_feedback_fresh(FeedbackSeen, now,
  LastFeedbackMs)，即已收到反馈且反馈年龄<=100ms；不用旧计数式看门狗，
  也不再维护 Wdg.isOnline 这类状态位副本。
- RawEcd：手动转对应轴时变化；Angle=RawEcd*2π/8192，单圈rad，非机械零点。
- RawSpe：电机轴rpm；Spe=RawSpe*2π/60，rad/s，不除15.76。
- RawTorque：为兼容旧名称保留，实际是原始转矩电流，不是Nm；Torque尚未标定。
- RawTemperature：摄氏度。

先连续运行再暂停看数据；暂停时计数不增长属正常。
确认两台在线、分别转动对应轴时只有相应编码器变化，再进行轴向/限位确认及单轴低输出测试。
ISR写入的多个字段不是原子快照，暂停调试可观察；后续闭环取样需增加一致性保护。

## 主机测试

独立配置tests目录，使用本机C++编译器而非ARM交叉编译器，构建后运行CTest。
覆盖反馈地址、长度、空指针、编码器范围、有符号值、失败保留结果、在线超时及计时回绕。
不替代真实CAN总线、接线、掉线和电机测试。
