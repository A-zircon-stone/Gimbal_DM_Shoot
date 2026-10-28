//
// Created by RedogJrome on 2026/3/6.
//

#ifndef INC_2026_BALANCED_GIMBAL_GIMBAL_HPP
#define INC_2026_BALANCED_GIMBAL_GIMBAL_HPP

#include <sys/types.h>

#include "Motor.hpp"
#include "Pid.hpp"
#include "PitchTilt.hpp"
#include "PitchDirectionTest.hpp"
#include "PitchImuHold.hpp"
#include "GimbalCommissioning.hpp"
#include "PitchSpeedTune.hpp"
#include "YawSpeedTune.hpp"
#include "VisionLink.hpp"
#include "RemoteControlProtection.hpp"

struct Cmd_t
{
    float Target_Vy;
    float Target_Vx;
    float Target_Vz;

    float Target_Yaw_Torque;//Yaw电机的扭矩


    uint8_t Flag_Firing_Mode;//发弹方式
    uint8_t Flag_Auto_Aim;//自瞄
    uint8_t Flag_RefreshUI;//随动
    uint8_t Flag_Spin;//小陀螺
    uint8_t Flag_SpeedUp;//加速


    uint8_t Flag_Clear;//请求清楚所有跳跃 上台阶等状态

    //串腿专用的
    uint8_t Flag_Reset_Leg;
    uint8_t Flag_Getting_Up;
    uint8_t Flag_Upstairs;
    uint8_t Flag_Jump;

    uint8_t Left_Fric_Status;
    uint8_t Right_Fric_Status;


    uint8_t Message_Spe[8];
    uint8_t Message_Motor[8];
    uint8_t Message_Cmd[8];
    uint8_t Message_Info[8];

    void Clear();
    void SendMessage();
    void SendProtect();
};



class Gimbal_t
{
    public:
    //这两个角度实时更新

    float Yaw_Angle;
    float Last_Yaw_Angle;
    float Yaw_Spe;
    float Yaw_Acc;
    int32_t Yaw_Round;


    float Pitch_Angle;
    float Last_Pitch_Angle;
    float Pitch_Spe;
    float Pitch_Acc;
    int32_t Pitch_Round;

    // 仅反馈阶段预览；后续控制必须另行检查时间戳、使能及输出方向。
    PitchAxisFeedback Pitch_Feedback{};
    uint32_t Pitch_FeedbackMs = 0;
    PitchTilt Pitch_Tilt{}; // 只由INS任务更新，保持逐帧解绕
    PitchDirectionTest Pitch_Test{}; // 只由GIMBAL任务推进
    PitchImuHold Pitch_Hold{}; // 与方向测试互斥，共用CAN1/ID2输出
    GimbalCommissioning Commissioning{};
    PitchSpeedTune Pitch_Speed_Tune{};
    YawSpeedTune Yaw_Speed_Tune{};
    VisionControl Vision_Control{};
    RemoteControlProtection Remote_Protection{};
    int16_t Yaw_Current_Command=0;
    bool Pitch_Boot_Position_Pending=true;
    bool Yaw_Boot_Position_Pending=true;
    int16_t Pitch_Current_Command=0; // 最终提交值，用于日志；不等于实测电流
    bool Vision_Pitch_Enabled=false;
    uint32_t Vision_Last_Sequence=0;
    uint32_t Pitch_Remote_LastMs=0;
    bool Pitch_Remote_TimingReady=false;
    void vital_gimbal_control();
    void Publish_Pitch_Feedback(const float q[4], float gyro_y_rad_s);


    float Target_Yaw_Angle_User;
    float Target_Pitch_Angle_User;
    float Target_Yaw_Angle_MiniPc;
    float Target_Pitch_Angle_MiniPc;

    float Target_Yaw_Spe_User;
    float Target_Pitch_Spe_User;
    float Target_Yaw_Spe_MiniPC;
    float Target_Pitch_Spe_MiniPC;

    float Left_Target_Fric_Spe;
    float Right_Target_Fric_Spe;

    Cmd_t Cmd;


    Motor_6020_t Motor_Yaw{0.0f};
    Motor_6020_t Motor_Pitch{0.0f};
    Motor_3508_t Motor_Left_Fric, Motor_Right_Fric;

    Pid_t Pitch_SpePid,Pitch_PosPid;
    Pid_t Yaw_SpePid,Yaw_PosPid;

    Pid_t Left_FricPid,Right_FricPid;

    bool Protect_Flag; //总保护标志位
    bool Manual_Protect_Flag; //手动保护标志位

    uint8_t Chassis_Status;
    uint8_t Robot_Color;
    uint8_t Head_Direction;
    uint8_t Refree_Elc;

    /*
     *STATE_OFF, //0
        STATE_LYING, //1
        STATE_GETTING_UP, //2
        STATE_STANDING, //3
     */
    uint16_t Robot_HP;

    Gimbal_t()
    {}
    void Init();
    void Update_Info();
    void Update_Control();
    void Update_Yaw();
    void Update_Pitch();
    void Update_Fric();

    void Fire_Detect();
    void Update_Fire_State();

    void Protect();

    void Clear();
    void Gimbal_Loop();

};
extern Gimbal_t Gimbal;

#endif //INC_2026_BALANCED_GIMBAL_GIMBAL_HPP
