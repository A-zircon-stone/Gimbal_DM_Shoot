# C板与standard_mpc视觉机联调

## 已实现

- C板原生USB CDC专用SP二进制协议：上行45字节，下行31字节，原CRC16算法。
- USART1 115200 8N1继续六通道角度FireWater波形（Yaw实际/目标/误差、Pitch实际/目标/误差，误差=target-actual），不向USB混发文本。
- 上行每5ms尝试发送最新有效INS姿态；USB忙时丢当次帧，不覆盖仍在传输的缓冲区、不积压。
- 四元数直接取INS.q并归一化，wxyz、机体系到惯性系；Yaw=INS.YawTotalAngle转rad，Pitch=INS.Roll转rad（抬头负）。
- Yaw速度采用现有Yaw控制器的陀螺仪姿态投影，Pitch速度采用gy；安装对应关系仍需实车核验。
- USB接收缓存半包/粘包，CRC失败后滑动1字节找SP；仅通过CRC及mode/fire/有限值检查的帧更新时间戳。
- 下行绝对目标直接进入现有双轴PID，Yaw保留连续圈数；Pitch按已有原始Roll范围[-35,+5]度钳位。
- 速度前馈rad/s转rpm，Pitch同时取负。叠加后仍用既有目标速度限幅和电流限幅。
- 加速度有独立电流模型系数接口；未标定，默认两个系数为0，不宣称已完成加速度前馈调试。
- 当前PID增益、限幅、输出步长、遥控速度保持用户文件中的值。

## 控制权与命令（USART1、CRLF结尾）

上电默认允许视觉控制，无需vision_enable。遥控在线且两轴反馈有效后自动进入双轴位置环并采用新的有效视觉目标；遥控未连接/断连时两轴电流归零，USB仍上报有效姿态但mode=0。

- `vcolor=1`：敌方红；`vcolor=2`：敌方蓝；0未知。默认未知，视觉按其YAML颜色配置兜底。
- `vmode=1`：自瞄；`vmode=2`：小符；`vmode=3`：大符；`vmode=0`/其他0..255值：空闲。
- `vision_enable`：从显式关闭视觉的状态恢复；默认上电已经允许。若所选为0则启动自瞄。丢弃使能前旧包，等待新的有效下行。
- `vision_disable`：退出视觉并保持退出时角度，交还遥控，上行mode回到0。
- `vision_status`：单次USART1状态，含enabled/active/up_mode/enemy/rx/age_ms/down_mode/reason和两轴目标。
- `stop`：停止两轴并撤销视觉许可，视觉帧不能自行重新启动。p:/y:和PID调参不再退出视觉；视觉许可期间p:/y:不覆盖目标，PID命令只改参数、不切换轴模式。需要手动调试时显式vision_disable。ystop仍可显式关闭Yaw并撤销视觉许可。

视觉active时摇杆不叠加目标。下行mode=0/超过200ms没有有效下行时，清除视觉控制，捕获当时实际角度保持并交还遥控；
遥控断连沿用原在线判定（原计数200次、控制周期2ms，约400ms），不新增阈值；一旦判离线立即两轴发0、清PID。重连时重新捕获当前位置并丢弃断连期间视觉包，等新mode=1有效帧恢复跟踪；显式stop/vision_disable不会被重连自动取消。后续新有效mode=1帧可按当前视觉许可恢复跟踪。200ms是首次联调暂定值，在VisionLinkConfig::RxTimeoutMs修改，需双方实测确认。
上行工作模式切换时清除旧视觉目标并建立新接收基线；不把mode=0报文中的零角度当回零命令。

## 首次上机顺序

1. 烧录build/Debug/2026_Balanced_Gimbal.elf。C板原生USB接视觉机，USART1接VOFA。
2. 视觉程序使用此USB串口（Linux一般/dev/ttyACM0），串口只由一个程序占用。
3. 遥控器保持在线。未启动视觉程序时，可运行 `python tools/vision_link_probe.py --port /dev/ttyACM0 --once` 只读首包。
4. 若脚本提示缺少依赖，安装pyserial。无硬件自检：`python tools/vision_link_probe.py --self-test`。
5. 启动standard_mpc，看First q received。遥控在线时默认上报mode=1自瞄；遥控未连接时上报mode=0空闲。
6. VOFA发vcolor=1或2（敌方颜色），vmode=1；默认已允许视觉，应进入AUTO_AIM，不需要再发enable。
7. VOFA发vision_status，rx应增长，有有效控制解时active=1。观察六通道角度波形，核对抬头负、左转正和连续Yaw。
8. vision_disable返回遥控；停止小电脑下行验证200ms退出。

如果尚无摄像头，可关闭视觉程序后，用probe脚本显式发测试目标，例如先vision_enable，再执行：
`python tools/vision_link_probe.py --port /dev/ttyACM0 --yaw-delta-deg 2 --seconds 2`
或`--pitch-delta-deg -2`抬头2度。脚本50Hz发送绝对目标，fire始终0，结束发送mode=0。默认无delta参数只读、不控制。
Windows在同一命令把设备路径换成实际原生USB端口，例如COM20；不要用USART1的VOFA端口。

## 当前边界

本次不驱动发射机构。所有上行发弹/打符完成计数为0；下行fire、shoot_count虽然解析保存，但不执行射击，完整打符射击握手尚未实现。
未接弹速测量，上行bullet_speed=0明确为未知，由视觉既有弹速兜底处理。后续需接实际测量值。
敌方颜色不是本方颜色；协议边界使用数值1/2。安装外参默认按当前IMU与云台校准关系，需双方实测验证方向。
协议无时间戳/会话号，电控不能识别视觉违反协议发送的跨模式旧结果；视觉端必须遵守其发送前切档检查。
离线回归不代替实车USB收发、频率、方向和目标跟踪验收。
