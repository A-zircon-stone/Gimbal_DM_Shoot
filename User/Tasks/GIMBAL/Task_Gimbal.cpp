//
// Created by RedogJrome on 2026/3/6.
//

#include "Task_Gimbal.hpp"
#include "cmsis_os.h"
#include "Gimbal.hpp"
#include "usart.h"
#include "DamiaoPitchRuntime.hpp"
#include "DamiaoYawRuntime.hpp"
#include "GimbalRemotePolicy.hpp"
#include "remotec.hpp"
#include "CANio.hpp"


void Task_Gimbal(void *argument)
{
    Gimbal.Init();
    Gimbal.Manual_Protect_Flag = true;
    bool autoaim_selected = false;
    for(;;)
    {
        TickType_t t = xTaskGetTickCount();

        // 达妙 Pitch 首轮：运行新的 IMU 角度闭环调试链路。
        // 旧 Gimbal_Loop 保留在工程中；不能把 6020 输出直接发给达妙电机。
        // 原Gimbal_Loop当前未运行，因此在此维持原有遥控器在线检测和拨杆模式解析。
        // 遥控器离线或不在遥控模式时，摇杆按0处理，保持已有目标。
        rc_arbitration();
        RC_Levers();
        taskENTER_CRITICAL();
        const bool dr16_selected = Controller == eDR16;
        const bool remote_online = dr16_selected && dr16.online;
        const auto remote = dr16.value();
        taskEXIT_CRITICAL();
        const uint8_t s1 = dr16_selected ? remote.rc.s[1] : gimbal_remote_policy::SwitchMiddle;
        const uint8_t s2 = dr16_selected ? remote.rc.s[0] : gimbal_remote_policy::SwitchMiddle;
        const auto selection = gimbal_remote_policy::select(remote_online, s1, s2);
        AimMode = selection.autoaim_mode ? eAutoaim : eManual;

        uint32_t vision_sequence;
        uint32_t vision_rx_ms;
        VisionReceivePacket2026 vision_packet;
        taskENTER_CRITICAL();
        vision_sequence = vision_rx_sequence;
        vision_rx_ms = vision_rx_tick_ms;
        vision_packet = vision_rv_pkt_2026;
        taskEXIT_CRITICAL();

        Gimbal.Vision_Control.set_remote_online(remote_online, vision_sequence);
        if (selection.autoaim_mode) {
            if (!autoaim_selected) {
                Gimbal.Vision_Control.set_mode(1, vision_sequence);
                Gimbal.Vision_Control.enable(vision_sequence);
            }
        } else if (autoaim_selected || Gimbal.Vision_Control.enabled) {
            Gimbal.Vision_Control.disable("MANUAL");
        }
        autoaim_selected = selection.autoaim_mode;
        const uint32_t now = HAL_GetTick();
        (void)Gimbal.Vision_Control.update(now, vision_sequence, vision_rx_ms, vision_packet);

        const bool vision_target_active = selection.autoaim_mode && Gimbal.Vision_Control.active;
        // 视觉协议Pitch沿用INS.Roll符号，达妙Pitch runtime以抬头为正。
        const float vision_pitch_rad = -Gimbal.Vision_Control.target.pitch;
        const float pitch_stick = CtrlMode == eRC && selection.remote_mode &&
                                  !selection.autoaim_mode ? portGetCh1() : 0.0f;
        damiao_pitch_runtime::step(now, pitch_stick, selection.protected_mode,
                                   vision_target_active, vision_pitch_rad);
        // 旧工程Yaw用右摇杆左右的反号：沿用原方向，具体安装可在Yaw配置中反转。
        const float yaw_stick = CtrlMode == eRC && selection.remote_mode &&
                                !selection.autoaim_mode ? -portGetCh0() : 0.0f;
        const float vision_yaw_rad = Gimbal.Vision_Control.target.yaw;
        damiao_yaw_runtime::step(now, yaw_stick, selection.protected_mode,
                                 vision_target_active, vision_yaw_rad);

        ShooterInput shooter_input{};
        shooter_input.remote_online = remote_online;
        shooter_input.s1 = s1;
        shooter_input.s2 = s2;
        shooter_input.feeder_stick = remote.rc.ch[3];
        shooter_input.vision_fire_allowed = vision_target_active &&
                                            Gimbal.Vision_Control.target.fire == 1;
        taskENTER_CRITICAL();
        shooter_input.left_online = Gimbal.Motor_Left_Fric.Wdg.isOnline == OnlineState_t::ONLINE;
        shooter_input.right_online = Gimbal.Motor_Right_Fric.Wdg.isOnline == OnlineState_t::ONLINE;
        shooter_input.left_speed_rpm = Gimbal.Motor_Left_Fric.Spe;
        shooter_input.right_speed_rpm = Gimbal.Motor_Right_Fric.Spe;
        shooter_input.feeder_speed_rpm = Gimbal.Motor_Feeder.speed_rpm;
        taskEXIT_CRITICAL();
        Gimbal.Shooter_Output = Gimbal.Shooter.step(shooter_input);
        const auto &shoot = Gimbal.Shooter_Output;
        Gimbal.Shooter_CanStatus = shooter_can_send_currents(shoot.left_current,
                                                           shoot.right_current,
                                                           shoot.feeder_current);
        FricCtrlMode = shoot.friction_enabled ? eFric_OPEN : eFric_SHUT;
        // Gimbal.Gimbal_Loop();

        vTaskDelayUntil(&t,2);
    }
}
