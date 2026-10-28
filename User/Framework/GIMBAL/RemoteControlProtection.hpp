#pragma once

// 沿用已有遥控在线检测；此处只执行断连后的双轴零输出与重连位置捕获。
struct RemoteControlProtection
{
    bool protected_now=true;
    bool resume_pitch=false,resume_yaw=false;
    void cancel_pitch_resume() { resume_pitch=false; }
    void cancel_yaw_resume() { resume_yaw=false; }
    template<class Pitch,class Yaw>
    bool step(bool online,Pitch &pitch,Yaw &yaw,bool &pitch_boot,bool &yaw_boot)
    {
        if(!online)
        {
            if(!protected_now)
            {
                resume_pitch=pitch.enabled && pitch.position_mode;
                resume_yaw=yaw.enabled && yaw.position_mode;
            }
            pitch.stop("RC_OFFLINE"); yaw.stop("RC_OFFLINE");
            protected_now=true;
            return false;
        }
        if(protected_now)
        {
            if(resume_pitch) pitch_boot=true;
            if(resume_yaw) yaw_boot=true;
            resume_pitch=false; resume_yaw=false;
        }
        protected_now=false;
        return true;
    }
};
