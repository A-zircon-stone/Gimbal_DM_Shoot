//
// Created by RedogJrome on 2026/3/6.
//

#include "Gimbal.hpp"
#include "INSTask.hpp"
#include "remotec.hpp"
#include "CANio.hpp"
#include "AutoAim.hpp"
#include "debugc.hpp"
#include "MDO.hpp"
#include "Framework/LQR/Lqr.hpp"
#include "VisionPitchTarget.hpp"

Gimbal_t Gimbal;

void Gimbal_t::vital_gimbal_control()
{
    if(!GimbalHardware::FeedbackOnly || !GimbalHardware::PitchDirectionTestEnabled) return;
    if(GimbalHardware::SerialTestOnly)
    {
        Pitch_Speed_Tune.stop();
        Yaw_Speed_Tune.stop(); Yaw_Current_Command=0;
        Pitch_Hold.stop();
        Commissioning.step(HAL_GetTick(),PitchTestRequest::Stop);
        Pitch_Current_Command=0;
        taskENTER_CRITICAL();
        debug.pitch_request=PitchTestRequest::None;
        debug.pitch_speed_gain_pending=0;
        taskEXIT_CRITICAL();
        (void)Cmd_Gimbal_Raw_Current(0,0);
        return;
    }
    if(GimbalHardware::PitchSpeedTuneMode)
    {
        taskENTER_CRITICAL();
        const uint32_t now=HAL_GetTick();
        rc_arbitration();
        RC_Levers();
        const bool remote_online=(Controller==eDR16 && dr16.online) || (Controller==eVT03 && vt03.online);
        const float remote_pitch=CtrlMode==eRC ? portGetCh1() : 0.0f;
        const float remote_yaw=CtrlMode==eRC ? -portGetCh0() : 0.0f;
        const float remote_dt=Pitch_Remote_TimingReady ?
            static_cast<uint32_t>(now-Pitch_Remote_LastMs)*0.001f : 0.0f;
        Pitch_Remote_LastMs=now;
        Pitch_Remote_TimingReady=true;
        const bool position_gain_updated=(debug.pitch_speed_gain_pending & 8)!=0;
        if(debug.pitch_speed_gain_pending & 1) Pitch_Speed_Tune.pid.SpdParam.Kp1=debug.pitch_speed_kp;
        if(debug.pitch_speed_gain_pending & 2) Pitch_Speed_Tune.pid.SpdParam.Ki1=debug.pitch_speed_ki;
        if(debug.pitch_speed_gain_pending & 4) Pitch_Speed_Tune.pid.SpdParam.Kd1=debug.pitch_speed_kd;
        if(debug.pitch_speed_gain_pending & 8) Pitch_Speed_Tune.pid.PosParam.Kp1=debug.pitch_position_kp;
        debug.pitch_speed_gain_pending=0;
        auto request=debug.pitch_request;
        const float position_delta=debug.pitch_delta_deg;
        if(request!=PitchTestRequest::Stop && static_cast<uint32_t>(now-debug.pitch_request_ms)>200)
            request=PitchTestRequest::None;
        if(request==PitchTestRequest::None && position_gain_updated)
            request=PitchTestRequest::PositionEnable;
        debug.pitch_request=PitchTestRequest::None;
        const uint8_t yaw_mask=debug.yaw_gain_pending;
        const float ykp=debug.yaw_speed_kp, yki=debug.yaw_speed_ki;
        const float ykd=debug.yaw_speed_kd, ykpp=debug.yaw_position_kp;
        debug.yaw_gain_pending=0;
        const bool yaw_stop=debug.yaw_stop_requested;
        debug.yaw_stop_requested=false;
        const bool yaw_delta_pending=debug.yaw_delta_pending;
        const float yaw_delta=debug.yaw_delta_deg;
        debug.yaw_delta_pending=false;
        const bool vision_disable=debug.vision_disable_requested;
        debug.vision_disable_requested=false;
        const bool vision_mode_pending=debug.vision_mode_pending;
        const uint8_t vision_mode=debug.vision_mode;
        debug.vision_mode_pending=false;
        if(debug.vision_color_pending)
        { Vision_Control.enemy_color=debug.vision_color; debug.vision_color_pending=false; }
        const uint32_t vision_sequence=vision_rx_sequence;
        const uint32_t vision_ms=vision_rx_tick_ms;
        const VisionReceivePacket2026 vision_packet=vision_rv_pkt_2026;
        const float yaw_deg=INS.YawTotalAngle;
        const float yaw_rate=YawSpeedTune::yaw_rate_rad(INS.Roll,INS.Pitch,INS.Gyro[Y_],INS.Gyro[Z_]);
        const bool yaw_valid=QEKF_INS.LastUpdateValid &&
            (GimbalHardware::PitchSpeedDryRun || Motor_Yaw.FeedbackSeen);
        const float roll_deg=INS.Roll;
        const float gyro_y=INS.Gyro[Y_];
        const bool valid=QEKF_INS.LastUpdateValid &&
            (GimbalHardware::PitchSpeedDryRun || Motor_Pitch.FeedbackSeen);
        taskEXIT_CRITICAL();
        Pitch_Hold.boot_target_pending=false;
        Vision_Pitch_Enabled=false;
        // 视觉只改变当前双轴控制器的目标，不启用旧自瞄/摩擦轮/底盘输出。
        const auto hold_current=[&]()
        {
            if(Pitch_Speed_Tune.enabled && Pitch_Speed_Tune.position_mode && std::isfinite(roll_deg))
            {
                Pitch_Speed_Tune.target_roll_deg=std::fmax(PitchSpeedTune::RollMinDeg,
                    std::fmin(PitchSpeedTune::RollMaxDeg,roll_deg));
                Pitch_Speed_Tune.pid.PID_Clear();
            }
            if(Yaw_Speed_Tune.enabled && Yaw_Speed_Tune.position_mode && std::isfinite(yaw_deg))
            { Yaw_Speed_Tune.target_yaw_deg=yaw_deg; Yaw_Speed_Tune.pid.PID_Clear(); }
        };
        bool held_this_frame=false;
        // 仅显式vision_disable/停车命令退出视觉。p:/y:/PID调参不再抢占控制权。
        const bool stop_requested=request==PitchTestRequest::Stop || yaw_stop;
        if(vision_disable || stop_requested)
        {
            if(Vision_Control.active) { hold_current(); held_this_frame=true; }
            Vision_Control.disable();
        }
        if(vision_mode_pending)
        {
            const bool switching=(vision_mode<=3 ? vision_mode : 0)!=Vision_Control.requested_mode;
            if(switching && Vision_Control.active) { hold_current(); held_this_frame=true; }
            Vision_Control.set_mode(vision_mode,vision_sequence);
        }
        Vision_Control.set_remote_online(remote_online,vision_sequence);
        if(request==PitchTestRequest::Stop)
        { Pitch_Boot_Position_Pending=false; Remote_Protection.cancel_pitch_resume(); Pitch_Speed_Tune.stop(); }
        if(yaw_stop)
        { Yaw_Boot_Position_Pending=false; Remote_Protection.cancel_yaw_resume(); Yaw_Speed_Tune.stop(); }
        // 参数更新始终保留；视觉许可下ykp=不再切回单速度模式。
        Yaw_Speed_Tune.apply_gains(yaw_mask,ykp,yki,ykd,ykpp,now,yaw_deg,!Vision_Control.enabled && remote_online);
        if(!Remote_Protection.step(remote_online,Pitch_Speed_Tune,Yaw_Speed_Tune,
                                  Pitch_Boot_Position_Pending,Yaw_Boot_Position_Pending))
        {
            Protect_Flag=true;
            Pitch_Current_Command=0; Yaw_Current_Command=0; AutoAim.Fire=0;
            Pitch_Remote_TimingReady=false;
            (void)Cmd_Gimbal_Raw_Current(0,0);
            return; // 遥控断连时，视觉包/串口命令都不能越过保护产生输出
        }
        Protect_Flag=false;
        // 默认视觉模式不处理p:/y:的位置覆盖，也不因调参隐式改变轴工作模式。
        const bool manual_control=!Vision_Control.enabled;
        // Pitch沿用代码中的调好参数；首次合法反馈时捕获当前位置，不自动回零。
        if(Pitch_Boot_Position_Pending)
        {
            if(manual_control && request!=PitchTestRequest::None) Pitch_Boot_Position_Pending=false;
            else if(valid)
            {
                Pitch_Speed_Tune.enable_position(now,roll_deg);
                Pitch_Boot_Position_Pending=false;
            }
        }
        if(request==PitchTestRequest::Stop) Pitch_Speed_Tune.stop();
        else if(manual_control && request==PitchTestRequest::Hold) Pitch_Speed_Tune.enable(now);
        else if(manual_control && request==PitchTestRequest::PositionEnable) Pitch_Speed_Tune.enable_position(now,roll_deg);
        else if(manual_control && request==PitchTestRequest::PitchDelta) Pitch_Speed_Tune.shift_position(position_delta);
        // 只在本次上电首次有效反馈时进入遥控位置模式；停止后不自动重启。
        // 显式ykp=/ykpp=仍优先决定本轮模式，保持原串口调参行为。
        if(Yaw_Boot_Position_Pending)
        {
            if(yaw_stop || (manual_control && (yaw_mask & 9))) Yaw_Boot_Position_Pending=false;
            else if(yaw_valid && std::isfinite(yaw_deg) && std::isfinite(yaw_rate))
            {
                Yaw_Speed_Tune.enable_position(now,yaw_deg);
                Yaw_Boot_Position_Pending=false;
            }
        }
        if(yaw_stop) Yaw_Speed_Tune.stop();
        else if(manual_control && yaw_delta_pending) Yaw_Speed_Tune.shift_position(yaw_delta);
        if(request==PitchTestRequest::VisionEnable && !vision_disable && !yaw_stop)
        {
            if(valid && yaw_valid)
            {
                Pitch_Speed_Tune.enable_position(now,roll_deg);
                Yaw_Speed_Tune.enable_position(now,yaw_deg);
                hold_current(); held_this_frame=true;
                Pitch_Boot_Position_Pending=false; Yaw_Boot_Position_Pending=false;
                Vision_Control.enable(vision_sequence);
            }
            else Vision_Control.disable("INVALID_FEEDBACK");
        }
        const bool axes_ready=Pitch_Speed_Tune.enabled && Yaw_Speed_Tune.enabled &&
            Pitch_Speed_Tune.position_mode && Yaw_Speed_Tune.position_mode && valid && yaw_valid;
        if(Vision_Control.enabled && !axes_ready)
        {
            if(Vision_Control.active) { hold_current(); held_this_frame=true; }
            Vision_Control.suspend(vision_sequence,"AXIS_NOT_READY");
        }
        const auto vision_action=axes_ready ? Vision_Control.update(now,vision_sequence,vision_ms,vision_packet) :
            VisionControl::Action::None;
        if(vision_action==VisionControl::Action::Hold)
        { hold_current(); held_this_frame=true; }
        float pitch_speed_ff=0,yaw_speed_ff=0,pitch_current_ff=0,yaw_current_ff=0;
        if(Vision_Control.active)
        {
            VisionAxisTargets mapped;
            if(map_vision_targets(Vision_Control.target,PitchSpeedTune::RollMinDeg,PitchSpeedTune::RollMaxDeg,mapped))
            {
                Pitch_Speed_Tune.target_roll_deg=mapped.pitch_deg;
                Yaw_Speed_Tune.target_yaw_deg=mapped.yaw_deg;
                pitch_speed_ff=mapped.pitch_speed_rpm; yaw_speed_ff=mapped.yaw_speed_rpm;
                pitch_current_ff=mapped.pitch_current_ff; yaw_current_ff=mapped.yaw_current_ff;
            }
        }
        else if(!held_this_frame)
        {
            if(request==PitchTestRequest::None) Pitch_Speed_Tune.update_remote_target(remote_pitch,remote_dt);
            if(!yaw_delta_pending && !yaw_stop) Yaw_Speed_Tune.update_remote_target(remote_yaw,remote_dt);
        }
        // 本轮不接射击；不能仅凭报文fire/shoot_count执行发射。
        AutoAim.Fire=0;
        Pitch_Speed_Tune.step(now,roll_deg,gyro_y,valid,!GimbalHardware::PitchSpeedDryRun,pitch_speed_ff,pitch_current_ff);
        Pitch_Current_Command=Pitch_Speed_Tune.command;
        Yaw_Speed_Tune.step(now,yaw_deg,yaw_rate,yaw_valid,yaw_speed_ff,yaw_current_ff);
        Yaw_Current_Command=Yaw_Speed_Tune.command;
        if(!GimbalHardware::PitchSpeedDryRun && Cmd_Gimbal_Raw_Current(Yaw_Current_Command,Pitch_Current_Command)!=HAL_OK)
        {
            Pitch_Speed_Tune.last_can_error=HAL_CAN_GetError(&hcan1);
            Pitch_Speed_Tune.stop("CAN_TX_FAILED"); Pitch_Current_Command=0;
            Yaw_Speed_Tune.stop("CAN_TX_FAILED"); Yaw_Current_Command=0;
            Vision_Control.disable("CAN_TX_FAILED");
            (void)Cmd_Gimbal_Raw_Current(0,0);
        }
        return;
    }
    if(GimbalHardware::CommissioningMode)
    {
        taskENTER_CRITICAL();
        const uint32_t now=HAL_GetTick();
        auto request=debug.pitch_request;
        if(request!=PitchTestRequest::Stop && static_cast<uint32_t>(now-debug.pitch_request_ms)>200)
            request=PitchTestRequest::None;
        debug.pitch_request=PitchTestRequest::None;
        debug.pitch_speed_gain_pending=0;
        taskEXIT_CRITICAL();
        Pitch_Hold.stop();
        Vision_Pitch_Enabled=false;
        if(GimbalCommissioning::ContinuousCurrentTest)
            Commissioning.step_continuous(request);
        else Commissioning.step(now,request);
        Pitch_Current_Command=Commissioning.pitch_current;
        if(Cmd_Gimbal_Raw_Current(Commissioning.yaw_current,Commissioning.pitch_current)!=HAL_OK)
        {
            Commissioning.step(now,PitchTestRequest::Stop);
            Pitch_Current_Command=0;
            (void)Cmd_Gimbal_Raw_Current(0,0);
        }
        return;
    }
    taskENTER_CRITICAL();
    const uint32_t now=HAL_GetTick();
    // 仅复用遥控输入选择/通道解析，不进入旧电机和底盘控制链路。
    rc_arbitration();
    RC_Levers();
    const float remote_pitch=CtrlMode==eRC ? portGetCh1() : 0.0f;
    const float remote_dt=Pitch_Remote_TimingReady ?
        static_cast<uint32_t>(now-Pitch_Remote_LastMs)*0.001f : 0.0f;
    Pitch_Remote_LastMs=now;
    Pitch_Remote_TimingReady=true;
    // 参数在控制任务中应用，避免接收中断改写正在计算的PID。
    if(debug.pitch_speed_gain_pending & 1) Pitch_Hold.pid.SpdParam.Kp1=debug.pitch_speed_kp;
    if(debug.pitch_speed_gain_pending & 2) Pitch_Hold.pid.SpdParam.Ki1=debug.pitch_speed_ki;
    if(debug.pitch_speed_gain_pending & 4) Pitch_Hold.pid.SpdParam.Kd1=debug.pitch_speed_kd;
    if(debug.pitch_speed_gain_pending & 8) Pitch_Hold.pid.PosParam.Kp1=debug.pitch_position_kp;
    debug.pitch_speed_gain_pending=0;
    auto request=debug.pitch_request;
    const float pitch_delta=debug.pitch_delta_deg;
    if(request!=PitchTestRequest::Stop && static_cast<uint32_t>(now-debug.pitch_request_ms)>200)
        request=PitchTestRequest::None;
    debug.pitch_request=PitchTestRequest::None;
    PitchTestInput in{};
    in.fresh=Pitch_Feedback.valid && static_cast<uint32_t>(now-Pitch_FeedbackMs)<=20 &&
        gm6020_feedback_fresh(Motor_Pitch.FeedbackSeen,now,Motor_Pitch.LastFeedbackMs) &&
        gm6020_feedback_fresh(Motor_Yaw.FeedbackSeen,now,Motor_Yaw.LastFeedbackMs);
    in.encoder=static_cast<uint16_t>(Motor_Pitch.RawEcd);
    in.rpm=Motor_Pitch.RawSpe;
    in.speed_rad_s=Pitch_Feedback.speed_rad_s;
    // 闭环按用户要求取消反馈超时停机。仍须已有合法IMU和编码器样本；
    // 不使用会因CAN超时清零的映射预览值，避免把零占位当成新测量。
    PitchHoldInput hold_input{
        Pitch_Tilt.valid && QEKF_INS.LastUpdateValid && Motor_Pitch.FeedbackSeen,
        -Pitch_Tilt.continuous_deg*0.017453292519943295f,
        -INS.Gyro[Y_],in.encoder};
    const uint32_t vision_sequence=vision_rx_sequence;
    const VisionReceivePacket2026 vision_packet=vision_rv_pkt_2026;
    taskEXIT_CRITICAL();
    // 手动请求优先；停止后不会自动重新回位。
    if(request!=PitchTestRequest::None) Pitch_Hold.boot_target_pending=false;
    else Pitch_Hold.start_boot_target(now,hold_input);
    if(request==PitchTestRequest::VisionEnable)
    {
        // 仅开放后续新包，不应用进入视觉模式前缓存的旧目标。
        Vision_Pitch_Enabled=true;
        Vision_Last_Sequence=vision_sequence;
        request=PitchTestRequest::Hold;
    }
    else if(request!=PitchTestRequest::None)
        Vision_Pitch_Enabled=false; // 手动停机/控制优先，视觉包不能重启
    if(request==PitchTestRequest::Stop)
    {
        Pitch_Hold.stop();
        Pitch_Test.step(now,PitchTestRequest::Stop,in);
    }
    else if(request==PitchTestRequest::Hold)
    {
        if(Pitch_Test.state==PitchDirectionTest::Running || Pitch_Test.state==PitchDirectionTest::Fault)
        {
            Pitch_Hold.fail();
            Pitch_Test.step(now,PitchTestRequest::Stop,in);
        }
        else
        {
            Pitch_Test.step(now,PitchTestRequest::Stop,in);
            Pitch_Hold.enable(now,hold_input);
        }
    }
    else if(request==PitchTestRequest::PitchDelta)
    {
        // 手动增量修改目标，不重置PID、不捕获新零位，不启动已关闭的闭环。
        Pitch_Hold.shift_target_degrees(pitch_delta);
    }
    else if(Pitch_Hold.state==PitchImuHold::Holding && request!=PitchTestRequest::None)
    {
        // 闭环中收到脉冲请求只停车，不允许叠加/切换到开环电流。
        Pitch_Hold.stop();
        Pitch_Test.step(now,PitchTestRequest::Stop,in);
    }
    else if(Pitch_Hold.state!=PitchImuHold::Holding)
    {
        if(Pitch_Hold.state!=PitchImuHold::Fault)
            Pitch_Test.step(now,request,in);
    }
    if(vision_sequence!=Vision_Last_Sequence)
    {
        Vision_Last_Sequence=vision_sequence;
        if(Vision_Pitch_Enabled)
        {
            if(vision_packet.mode==0)
                Pitch_Hold.stop();
            else
            {
                float target=0;
                if(vision_pitch_target(vision_packet,target) &&
                   Pitch_Test.state!=PitchDirectionTest::Running &&
                   Pitch_Test.state!=PitchDirectionTest::Fault &&
                   Pitch_Hold.enable(now,hold_input))
                    Pitch_Hold.set_target(target);
            }
        }
    }
    // 有摇杆输入才接管目标；串口请求优先，关闭/故障状态不自动启动。
    if(request==PitchTestRequest::None && Pitch_Hold.update_remote_target(remote_pitch,remote_dt))
        Vision_Pitch_Enabled=false;
    Pitch_Hold.step(now,hold_input);
    Pitch_Current_Command=Pitch_Hold.state==PitchImuHold::Holding ? Pitch_Hold.command :
        (Pitch_Hold.state==PitchImuHold::Fault ? 0 : Pitch_Test.command);
    if((Pitch_Test.bus_owned || Pitch_Hold.bus_owned) &&
        Cmd_Pitch_Direction_Test(Pitch_Current_Command)!=HAL_OK)
    {
        Pitch_Test.fail();
        Pitch_Hold.fail();
        Pitch_Current_Command=0;
        (void)Cmd_Pitch_Direction_Test(0);
    }
}

void Gimbal_t::Publish_Pitch_Feedback(const float q[4], float gyro_y_rad_s)
{
    if (!GimbalHardware::FeedbackOnly) return; // 不改变旧控制分支
    update_pitch_tilt(&Pitch_Tilt, q);
    // 同一临界区捕获CAN编码器、在线时间并发布结果，防止中断混帧。
    taskENTER_CRITICAL();
    const uint32_t now = HAL_GetTick();
    Pitch_Feedback = map_pitch_feedback(Pitch_Tilt, gyro_y_rad_s,
        static_cast<uint16_t>(Motor_Pitch.RawEcd),
        gm6020_feedback_fresh(Motor_Pitch.FeedbackSeen, now, Motor_Pitch.LastFeedbackMs));
    Pitch_FeedbackMs = now;
    if (Pitch_Feedback.valid)
    {
        Pitch_Angle = Pitch_Feedback.angle_rad;
        Pitch_Spe = Pitch_Feedback.speed_rad_s;
    }
    taskEXIT_CRITICAL();
}

void Cmd_t::Clear()
{
    Target_Vy = 0.0f;
    Target_Vx = 0.0f;
    Target_Vz = 0.0f;

    //通用接口
    Flag_Spin = 0;
    Flag_RefreshUI = 0;
    Flag_Auto_Aim = 0;


    //串腿专用的
    Flag_Reset_Leg = 0;
    Flag_Getting_Up = 0;
    Flag_Upstairs = 0;
    Flag_Jump = 0;

    Flag_Firing_Mode = 0;//发弹方式
    Flag_Spin = 0;//小陀螺
    Flag_SpeedUp = 0;//加速


    Flag_Clear = 0;//请求清楚所有跳跃 上台阶等状态

    memset(Message_Info, 0, sizeof(Message_Info));
    memset(Message_Spe, 0, sizeof(Message_Spe));
    memset(Message_Cmd, 0, sizeof(Message_Cmd));
    memset(Message_Motor,0, sizeof(Message_Motor));
}
void Cmd_t::SendMessage()
{
    static uint8_t Send_Round;
    memset(Message_Spe, 0, sizeof(Message_Spe));
    memset(Message_Motor,0, sizeof(Message_Motor));
    memset(Message_Info, 0, sizeof(Message_Spe));
    memset(Message_Cmd, 0, sizeof(Message_Spe));

    //底盘速度部分
    float tmp_spe,tmp_omega,tmp_vz;

    tmp_spe = Target_Vy * 100.0f;
    tmp_omega = Target_Vx * 100.0f;
    tmp_vz = Target_Vz * 100.0f;

    auto spe_int = static_cast<int16_t>(tmp_spe);
    auto omega_int = static_cast<int16_t>(tmp_omega);
    auto vz_int = static_cast<int16_t>(tmp_vz);


    Message_Spe[0] = static_cast<uint8_t>(spe_int >> 8);
    Message_Spe[1] = static_cast<uint8_t>(spe_int);
    Message_Spe[2] = static_cast<uint8_t>(omega_int >> 8);
    Message_Spe[3] = static_cast<uint8_t>(omega_int);
    Message_Spe[4] = static_cast<uint8_t>(vz_int >> 8);
    Message_Spe[5] = static_cast<uint8_t>(vz_int);
    Message_Spe[6] = Flag_SpeedUp;//是否加速
    //Message_Spe[7] = AutoAim.Mode;//自瞄
    if (AutoAim.Mode == 1 && AimMode == eAutoaim) Message_Spe[7] = 1;
    else Message_Spe[7] = 0;

    //YAW电机和拨弹电机
    float tmp_yaw_torque;

    tmp_yaw_torque = Target_Yaw_Torque * 100.0f;
    //usart_printf()

    auto yaw_torque_int = static_cast<int16_t>(tmp_yaw_torque);

    Message_Motor[0] = static_cast<uint8_t>(yaw_torque_int >> 8);
    Message_Motor[1] = static_cast<uint8_t>(yaw_torque_int);
    Message_Motor[2] = Left_Fric_Status;
    Message_Motor[3] = Right_Fric_Status;

    Message_Motor[4] = Flag_Firing_Mode;//单发还是连发 1是单发 2是连发
    Message_Motor[5] = Flag_RefreshUI;//UI刷新
    Message_Motor[6] = FricCtrlMode;

    Message_Motor[7] = 9;//检验位
    //串腿专用命令
    /*
    uint8_t Flag_Protect;
    uint8_t Flag_Lying;
    uint8_t Flag_Getting_Up;
    uint8_t Flag_Upstairs;
    uint8_t Flag_Jump;
    */
    Message_Cmd[0] = Gimbal.Protect_Flag;
    Message_Cmd[1] = Flag_Reset_Leg;
    Message_Cmd[2] = Flag_Getting_Up;
    Message_Cmd[3] = Flag_Upstairs;
    Message_Cmd[4] = Flag_Jump;
    Message_Cmd[5] = Flag_Spin;
    Message_Cmd[6] = Flag_Clear;
    Message_Cmd[7] = 9;//检验位置

    //
    Send_Round ++;
    if (Send_Round == 8) Send_Round = 0;
    if (Send_Round % 3 == 0)
    {
        SendMessage_8(&hcan2,0x401,Message_Spe);
        memset(Message_Spe, 0, sizeof(Message_Spe));
    }
    if (Send_Round % 3 == 1)
    {
        SendMessage_8(&hcan2,0x402,Message_Motor);
        memset(Message_Motor, 0, sizeof(Message_Motor));

    }
    if (Send_Round % 3 == 2)
    {
        SendMessage_8(&hcan2,0x403,Message_Cmd);
        memset(Message_Cmd, 0, sizeof(Message_Cmd));
    }
}

void Cmd_t::SendProtect() //保护命令
{
    memset(Message_Info, 0, sizeof(Message_Spe));
    memset(Message_Spe, 0, sizeof(Message_Spe));
    memset(Message_Cmd, 0, sizeof(Message_Spe));

    Message_Spe[7] = 4;
    Message_Motor[7] = 4;
    Message_Cmd[7] = 4;

    SendMessage_8(&hcan2,0x401,Message_Spe);
    SendMessage_8(&hcan2,0x402,Message_Motor);
    SendMessage_8(&hcan2,0x403,Message_Cmd);
}

void Gimbal_t::Init()
{
    // 仅反馈阶段：不初始化旧控制参数、不发送旧底盘/电机命令。
    if (GimbalHardware::FeedbackOnly)
    {
        Protect_Flag = true;
        Manual_Protect_Flag = true;
        return;
    }
    Yaw_PosPid.Init(11.0f,0.0f,0.0f);
    Yaw_SpePid.Init(2.0f,0.0f,0.0f);

    //Pitch_PosPid.Init(30.0f,0.0f,0.0f);
    //Pitch_SpePid.Init(0.5f,0.0f,0.0f);

    Pitch_PosPid.Init(40.0f,0.0f,0.0f);
    Pitch_SpePid.Init(1.4f,0.01f,0.0f);

    Left_FricPid.Init(0.0031,0.0f,0.000010f);
    Right_FricPid.Init(0.0031f,0.0f,0.000010f);

    Yaw_LQR.Q[0] = 2500;
    Yaw_LQR.Q[1] = 1.0f;
    Yaw_LQR.Init(15.0f / 1000.0f,0.0f,0.002f);
    Pitch_LQR.Init(0.018f,0.0f,0.002f);

    //MDO部分
    Pitch_MDO.Init(0.012f,0.0f,0.0f,30.0f);
    Yaw_MDO.Init(0.053f,0.1f,0.0f,1.0f);

    Left_Fric_MDO.Init(0.0f,0.0f,0.0f,1.0f);
    Right_Fric_MDO.Init(0.0f,0.0f,0.0f,1.0f);

    Manual_Protect_Flag = true;
    Protect();
}

void Gimbal_t::Update_Info()
{
    // Yaw Pitch部分
    static float Tmp_Yaw_Angle, Tmp_Pitch_Angle;

    if (INS.Yaw / 180.0f * PI - Tmp_Yaw_Angle > PI) Yaw_Round --;
    if (INS.Yaw / 180.0f * PI - Tmp_Yaw_Angle < -PI) Yaw_Round ++;

    if (-INS.Roll / 180.0f * PI - Tmp_Pitch_Angle > PI) Pitch_Round --;
    if (-INS.Roll / 180.0f * PI - Tmp_Pitch_Angle < -PI) Pitch_Round ++;

    Yaw_Angle = LowPassFilter(INS.Yaw / 180.0f * PI + static_cast<float>(Yaw_Round) * PI * 2.0f , Yaw_Angle,1.0f);
    Pitch_Angle = LowPassFilter(-INS.Roll / 180.0f * PI + static_cast<float>(Pitch_Round) * PI * 2.0f,Pitch_Angle,1.0f);

    Yaw_Acc = LowPassFilter((INS.Gyro[2] - Yaw_Spe) / 0.002f,Yaw_Spe,0.8f);
    Yaw_Spe = LowPassFilter(INS.Gyro[2], Yaw_Spe, 1.0f);

    Pitch_Acc = LowPassFilter((-INS.Gyro[1] - Pitch_Spe)/0.002f,Pitch_Acc,0.8f);
    Pitch_Spe = -INS.Gyro[1];

    Last_Yaw_Angle = Yaw_Angle;
    Last_Pitch_Angle = Pitch_Angle;

    Tmp_Yaw_Angle = INS.Yaw / 180.0f * PI;
    Tmp_Pitch_Angle = -INS.Roll / 180.0f * PI;
}

void Gimbal_t::Clear()//?
{
    Yaw_PosPid.Clear();
    Yaw_SpePid.Clear();
    Pitch_SpePid.Clear();
    Pitch_PosPid.Clear();
}

void Gimbal_t::Update_Control()
{
    static constexpr float Rc_Sensitivity = 0.05f;
    static constexpr float Key_Sensitivity = 0.04f;


    if (CtrlMode == eRC) //如果是遥控器控制 那就得老实写switch
    {
        if (Protect_Flag == false) //如果不是保护 可以有速度
        {
            Target_Yaw_Spe_User = -portGetCh0() * Rc_Sensitivity;//右摇杆左右
            Target_Pitch_Spe_User = portGetCh1() * Rc_Sensitivity;//右摇杆上下
        }
        else
        {
            Target_Yaw_Spe_User = 0;//右摇杆左右
            Target_Pitch_Spe_User = 0;//右摇杆上下
        }
        if (Controller == eDR16) //传统遥控器
        {
            Target_Yaw_Spe_User= -portGetCh0() * Rc_Sensitivity;
            Target_Pitch_Spe_User= portGetCh1() * Rc_Sensitivity;

            //普通接口
            Cmd.Target_Vy = portSetVy();
            Cmd.Target_Vx = -portSetVx();
            Cmd.Target_Vz = 0.0f;

            switch (dr16.value().rc.s[0]) // 右侧拨杆
            {
                case 1: // 上拨
                    Cmd.Flag_Spin = 1;
                    //Cmd.Flag_Upstairs = 1;
                    //AimMode = eAutoaim;
                    Cmd.Flag_Getting_Up = 0;

                    break;
                case 3: // 拨到中间
                    AimMode = eManual;
                    Cmd.Flag_Spin = 0;
                    Cmd.Flag_Getting_Up = 1;

                    break;
                case 2: // 下拨
                    Cmd.Flag_Spin = 0;
                    Cmd.Flag_Getting_Up = 0;
                    Cmd.Target_Vz = -portGetWheel();
                    break;
                default:
                    break;
            }
            // 摩擦轮控制
            if (dr16.value().rc.s[1] == 1) // 左拨杆上拨
            {
                FricCtrlMode = eFric_OPEN;
            }
            if (dr16.value().rc.s[1] == 3) // 左拨杆置中
            {
                FricCtrlMode = eFric_SHUT;
            }
            if (dr16.value().rc.s[1] == 2) // 左拨杆置下 遥控器模式
            {
            }
            if (dr16.value().rc.ch[4] <= -600) Manual_Protect_Flag = 0;

        }
        else if (Controller == eVT03) // 图传链路
        {
            if (vt03.value().rc.pause.Is_Click_Once)
            {
                Manual_Protect_Flag = 1 - Manual_Protect_Flag;
                vt03.value().rc.pause.Is_Click_Once = 0;
            }
            Target_Yaw_Spe_User= -portGetCh0() * Rc_Sensitivity * 0.8f;
            Target_Pitch_Spe_User= portGetCh1() * Rc_Sensitivity * 0.6f;

            Cmd.Target_Vy = portSetVx();
            Cmd.Target_Vx = -portSetVy();
            Cmd.Target_Vz = 0.0f;
            if (vt03.value().rc.mode_sw == 1) // 中间 做动作
            {
                if (vt03.value().rc.fn_1.Is_Click_Once)//左上小陀螺
                {
                    if (Gimbal.Chassis_Status == 3) Cmd.Flag_Spin = 1 - Cmd.Flag_Spin;
                    else Cmd.Flag_Spin = 0;
                    vt03.value().rc.fn_1.Is_Click_Once = 0;
                }
                if (vt03.value().rc.fn_2.Now_State)//右上起立
                {
                    Cmd.Flag_Getting_Up = 1;
                }
                else Cmd.Flag_Getting_Up = 0;
                if (vt03.value().rc.trigger.Now_State)
                {
                    Cmd.Flag_Upstairs = 1;
                }
                else Cmd.Flag_Upstairs = 0;
                Cmd.Target_Vz = portGetWheel();
            }
            else if (vt03.value().rc.mode_sw == 2) //右边 弹丸
            {
                if (vt03.value().rc.fn_1.Is_Click_Once)//左上开关摩擦轮
                {
                    if (FricCtrlMode == eFric_SHUT) FricCtrlMode = eFric_OPEN;
                    else FricCtrlMode = eFric_SHUT;

                    vt03.value().rc.fn_1.Is_Click_Once = 0;
                }
                if (vt03.value().rc.fn_2.Is_Click_Once)//右上切换自瞄
                {
                    if (AimMode == eManual)
                    {
                        AimMode = eAutoaim;
                        AutoAim.Symbol = 0;
                    }
                    else if (AimMode == eAutoaim)
                    {
                        if (AutoAim.Symbol == 0) AutoAim.Symbol = 1;
                        else if (AutoAim.Symbol == 1)
                        {
                            AutoAim.Symbol = 0;
                            AimMode = eManual;
                        }
                    }
                    vt03.value().rc.fn_2.Is_Click_Once = 0;
                }
            }
            //打弹丸
        }
        if (AutoAim.Symbol == 0)
        {
            if ((dr16.value().rc.ch[4] >= 600 || vt03.value().rc.trigger.Now_State == 1)  && FricCtrlMode == eFric_OPEN && ((AutoAim.Fire == 1 && AimMode == eAutoaim) || AimMode == eManual)) Cmd.Flag_Firing_Mode = 2;
            else Cmd.Flag_Firing_Mode = 0;
        }
        else if (AutoAim.Symbol == 1)
        {
            if ((dr16.value().rc.ch[4] >= 600 || vt03.value().rc.trigger.Now_State == 1)  && FricCtrlMode == eFric_OPEN && ((AutoAim.Fire == 2 && AimMode == eAutoaim && AutoAim.shoot_count_From_MiniPC != AutoAim.shoot_count_Now))) Cmd.Flag_Firing_Mode = 1;
            else Cmd.Flag_Firing_Mode = 0;

        }
    }

    else // 如果是键鼠控制 那就可以直接用接口
    {
        Target_Yaw_Spe_User= -portGetMouseX() * Key_Sensitivity;
        Target_Pitch_Spe_User= portGetMouseY() * Key_Sensitivity;
        //普通接口
        Cmd.Target_Vy = portSetVy() * 660.0f;
        Cmd.Target_Vx = portSetVx() * 660.0f;
        Cmd.Target_Vz = portGetMouseZ();

        if (portGetMouseRight() && AutoAim.Mode == 1) //如果右键摁下了 并且要控制
        {
            AimMode = eAutoaim;
            if (AutoAim.Fire == 1 && portGetMouseLeft()) Cmd.Flag_Firing_Mode = 2;
            else Cmd.Flag_Firing_Mode = 0;
        }
        else
        {
            AimMode = eManual;
            if (portGetMouseLeft()) Cmd.Flag_Firing_Mode = 2; //左键打弹丸
            else Cmd.Flag_Firing_Mode = 0;
        }

        if (portGetKeyQClick())
        {

        }


        if (portGetKeyShift()) //加速
        {
            Cmd.Flag_SpeedUp = 1;
        }
        else Cmd.Flag_SpeedUp = 0;

        if (portGetKeyCtrl()) //小陀螺
        {
            Cmd.Flag_Spin = 1;
        }
        else Cmd.Flag_Spin = 0;

        if (portGetKeyR()) //起立
        {
            Cmd.Flag_Getting_Up = 1;
        }
        else Cmd.Flag_Getting_Up = 0;

        if (portGetKeyVClick()) //
        {


        }

        if (portGetKeyB()) //重画ui
        {
            Cmd.Flag_RefreshUI = 1;
        }
        else Cmd.Flag_RefreshUI = 0;

        if (portGetKeyC()) //一键复位腿长（飞坡用）
        {
            Cmd.Flag_Reset_Leg = 1;
        }
        else Cmd.Flag_Reset_Leg = 0;

        //串腿接口
        if (portGetKeyEClick()) //开关摩擦轮
        {
            if (FricCtrlMode == eFric_OPEN) FricCtrlMode =eFric_SHUT;
            else if (FricCtrlMode == eFric_SHUT) FricCtrlMode =eFric_OPEN;
        }
        static uint16_t Upstair_Tick;

        if (portGetKeyF()) //磕上台阶
        {
            Cmd.Flag_Upstairs = 1;
        }
        else Cmd.Flag_Upstairs = 0;

        static uint32_t Protect_Tick;
        static uint32_t Enable_Tick;

        if (portGetKeyX())
        {
            if (!portGetKeyZ())
            {
                Protect_Tick ++;
                Enable_Tick = 0;
                if (Protect_Tick >=200) Manual_Protect_Flag = true;
            }
            else
            {
                Enable_Tick ++;
                Protect_Tick = 0;
                if (Enable_Tick >=200) Manual_Protect_Flag = false;
            }
                //注意 这个地方是长按X进保护 长按ZX是出保护
        }
    }
}

void Gimbal_t::Update_Yaw() //处理Yaw部分
{
    // 这是没有自瞄开火的指令情况下
    if ((Chassis_Status == 3 || Chassis_Status == 1) && Protect_Flag == false && Head_Direction == 0)//也就是standing情况下
    {
        if (AimMode== eManual || (AimMode == eAutoaim && AutoAim.Mode == 0))
        {
            Target_Yaw_Angle_User += Target_Yaw_Spe_User * 0.002f;
            Limit(Target_Yaw_Angle_User,Yaw_Angle - PI/2,Yaw_Angle + PI/2);

            Yaw_PosPid.Cal(Target_Yaw_Angle_User, Yaw_Angle, 100.0f);
            Yaw_SpePid.Cal(Yaw_PosPid.Output,Yaw_Spe,100.0f);

            Target_Yaw_Angle_MiniPc = Yaw_Angle;
            Cmd.Target_Yaw_Torque = Yaw_SpePid.Output - Yaw_MDO.Step(Yaw_Angle,Cmd.Target_Yaw_Torque);

        }
        else if (Protect_Flag == false && AutoAim.Mode == 1 && Head_Direction == 0)//有自瞄
        {
            Target_Yaw_Spe_MiniPC = AutoAim.Yaw_Spe;
            Target_Yaw_Angle_MiniPc = AutoAim.Yaw_Angle + AutoAim.Yaw_Spe * 4.0f/ 1000.0f;
            Target_Yaw_Angle_User = Target_Yaw_Angle_MiniPc;

            float tau_ff = AutoAim.Yaw_Acc * Yaw_LQR.J + AutoAim.Yaw_Spe * Yaw_LQR.b;
            //Cmd.Target_Yaw_Torque = Yaw_LQR.Step(Yaw_Angle,Yaw_Spe,Target_Yaw_Angle_MiniPc,Target_Yaw_Spe_MiniPC) + tau_ff - Yaw_MDO.Step(Yaw_Angle,Cmd.Target_Yaw_Torque);
            Cmd.Target_Yaw_Torque = Yaw_LQR.Step(Yaw_Angle,Yaw_Spe,Target_Yaw_Angle_MiniPc,Target_Yaw_Spe_MiniPC) + tau_ff ;

            Target_Yaw_Angle_User = Yaw_Angle;
        }
    }
    else
    {
        Target_Yaw_Angle_User = Yaw_Angle;
        Target_Yaw_Angle_MiniPc = Yaw_Angle;
        Cmd.Target_Yaw_Torque =0;
    }
}

void Gimbal_t::Update_Pitch() //处理Pitch部分
{
    if ((Chassis_Status == 3 || Chassis_Status == 1) && Protect_Flag == false && Head_Direction == 0)//也就是standing情况下
    {
        static float Last_tau;
        if (AimMode == eManual || (AimMode == eAutoaim && AutoAim.Mode == 0) )
        {
            Target_Pitch_Angle_User = Limit(Target_Pitch_Angle_User + Target_Pitch_Spe_User * 0.002f,PITCH_ANGLE_MIN,PITCH_ANGLE_MAX);

            Pitch_PosPid.Cal(Target_Pitch_Angle_User, Pitch_Angle, 100.0f);
            Pitch_SpePid.Cal(Pitch_PosPid.Output, Pitch_Spe, 100.0f);

            Motor_Pitch.Set_Target_Torque(Pitch_SpePid.Output - Pitch_MDO.Step(Pitch_Angle,Last_tau));

            Target_Pitch_Angle_MiniPc = Pitch_Angle;
            Last_tau=  Pitch_SpePid.Output - Pitch_MDO.Step(Pitch_Angle,Last_tau);
        }
        else if (Protect_Flag == false && AutoAim.Mode == 1 && Head_Direction == 0)//有自瞄
        {
            Target_Pitch_Angle_MiniPc = -AutoAim.Pitch_Angle;
            Pitch_PosPid.Cal(-(AutoAim.Pitch_Angle + AutoAim.Pitch_Spe * 4.0f / 1000.0f), Pitch_Angle, 100.0f);
            Pitch_SpePid.Cal(Pitch_PosPid.Output, Pitch_Spe, 100.0f);

            Motor_Pitch.Set_Target_Torque(Pitch_SpePid.Output - Pitch_MDO.Step(Pitch_Angle,Last_tau));
            Target_Pitch_Angle_User = Pitch_Angle;

        }
    }
    else
    {
        Target_Pitch_Angle_User = Pitch_Angle;
        Target_Pitch_Angle_MiniPc = Pitch_Angle;
        Motor_Pitch.Set_Target_Torque(0.0f);
    }
}

void Gimbal_t::Update_Fric() //处理摩擦轮部分
{
    //注意 摩擦轮的转速单位都是rpm
    static constexpr float Fric_Step = 3.0f;
    static constexpr float Fric_Threshold = 5800.0f;
    static float Last_Tau_Left,Last_Tau_Right;


    //if ( FricCtrlMode == eFric_OPEN && Chassis_Status == 3 && Protect_Flag == false) //如果打开摩擦轮 并且当前是站立情况
    if ( FricCtrlMode == eFric_OPEN) //如果打开摩擦轮
    {
        Left_Target_Fric_Spe += Fric_Step;
        Right_Target_Fric_Spe += Fric_Step;

        if (Left_Target_Fric_Spe > Fric_Threshold) Left_Target_Fric_Spe = Fric_Threshold;
        if (Right_Target_Fric_Spe > Fric_Threshold) Right_Target_Fric_Spe = Fric_Threshold;

        Left_FricPid.Cal(Left_Target_Fric_Spe, Motor_Left_Fric.Spe, 100.0f);
        Right_FricPid.Cal(-Right_Target_Fric_Spe, Motor_Right_Fric.Spe, 100.0f);

        Motor_Left_Fric.Set_Target_Torque(Left_FricPid.Output);
        Motor_Right_Fric.Set_Target_Torque(Right_FricPid.Output);
    }
    else
    {
        Left_Target_Fric_Spe = Motor_Left_Fric.Spe;
        Right_Target_Fric_Spe = -Motor_Right_Fric.Spe;
        Left_FricPid.Cal(0.0f, Motor_Left_Fric.Spe, 0.03f);
        Right_FricPid.Cal(0.0f, Motor_Right_Fric.Spe, 0.03f);

        Motor_Left_Fric.Set_Target_Torque(Left_FricPid.Output);
        Motor_Right_Fric.Set_Target_Torque(Right_FricPid.Output);
    }

    if (Motor_Left_Fric.Wdg.isOnline == OnlineState_t::ONLINE) //左边摩擦轮在线
    {
        if (fabs(Motor_Left_Fric.Spe) < 4000.0f) Cmd.Left_Fric_Status = 1; //中间速度过程
        else Cmd.Left_Fric_Status = 2; // 可以打弹丸
    }
    else Cmd.Left_Fric_Status = 0;

    if (Motor_Right_Fric.Wdg.isOnline == OnlineState_t::ONLINE) //右边摩擦轮在线
    {
        if (fabs(Motor_Right_Fric.Spe) < 4000.0f) Cmd.Right_Fric_Status = 1; //中间速度过程
        else Cmd.Right_Fric_Status = 2; //可以打弹丸
    }
    else Cmd.Right_Fric_Status = 0;
}

void Gimbal_t::Fire_Detect()//检测打出去没有
{
    static uint32_t tick;
    if (AutoAim.Symbol == 1 && fabs(Motor_Left_Fric.Spe) > 5000.0f && fabs(Motor_Right_Fric.Spe) > 5000.0f) //在单发
    {
        if (fabs(Motor_Left_Fric.Spe - 5800.0f) >120.0f
            && fabs(Motor_Right_Fric.Spe + 5800.0f) > 120.0f
            && tick >80
            )
        {
            tick = 0;
            AutoAim.shoot_count_Now ++;
        }
    }
    tick++;
    if (tick > 1000) tick = 1000;
}


void Gimbal_t::Protect()
{
    Cmd.Clear();
    Cmd.SendProtect();
    AimMode = eManual;
    FricCtrlMode = eFric_SHUT;

    Target_Pitch_Angle_User = Pitch_Angle;
    Target_Pitch_Angle_MiniPc = Pitch_Angle;
    Pitch_PosPid.Clear();
    Pitch_SpePid.Clear();

    Target_Yaw_Angle_User = Yaw_Angle;
    Target_Yaw_Angle_MiniPc = Yaw_Angle;
}

void Gimbal_t::Gimbal_Loop()
{
    if (GimbalHardware::FeedbackOnly)
    {
        taskENTER_CRITICAL();
        const uint32_t now = HAL_GetTick();
        if (static_cast<uint32_t>(now - Pitch_FeedbackMs) > 20 ||
            !gm6020_feedback_fresh(Motor_Pitch.FeedbackSeen, now, Motor_Pitch.LastFeedbackMs))
        {
            Pitch_Feedback.valid = false;
            Pitch_Feedback.inside_soft_limits = false;
            Pitch_Feedback.allow_raise = false;
            Pitch_Feedback.allow_lower = false;
        }
        taskEXIT_CRITICAL();
        vital_gimbal_control();
        if(!GimbalHardware::PitchSpeedTuneMode) Protect_Flag = true;
        return; // 不执行旧遥控/自瞄/PID/MDO/摩擦轮输出链路
    }
    rc_arbitration();  // 遥控器在线检测与仲裁
    RC_Levers();       // 遥控器模式解析
    Update_Info(); //解析云台角度数据
    Fire_Detect();
    if (Controller == eNone ||
        Manual_Protect_Flag ==true ||
        Robot_HP == 0
        || Refree_Elc == 0) Protect_Flag = true; //仲裁的结果是没有遥控器在线 或者手动进入了保护模式
    else Protect_Flag = false;
    Update_Control();

    Update_Yaw();
    Update_Pitch();
    Update_Fric();

    if (Protect_Flag == false) Cmd.SendMessage();
    else Protect();
}
