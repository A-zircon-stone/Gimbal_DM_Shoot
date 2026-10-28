#include "PrintTelemetry.hpp"
#include <cassert>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

int main()
{
    static_assert(PrintUart1FireWater, "默认USART1使用精简VOFA输出");
    PrintTelemetry d{};
    d.imu_fresh=d.pitch_y_valid=d.pitch_online=true;
    d.pitch_control_deg=-70.25f; d.pitch_target_deg=-72.5f;
    d.pitch_current_command=1000; d.pitch_current_feedback=950;
    d.pitch_encoder=5121; d.pitch_speed_deg_s=-0.5f; d.pitch_hold_state=1;
    char out[PrintTelemetryCapacity];
    // 控制器连续Yaw跨180度，Pitch采用原始Roll符号。
    d.yaw_control_deg=181.25f; d.yaw_target_deg=185.5f;
    const auto angle_size=format_gimbal_angles(out,sizeof(out),d);
    assert(angle_size==std::strlen(out));
    assert(std::strcmp(out,"angles:181.25,185.50,4.25,-70.25,-72.50,-2.25\r\n")==0);
    assert(format_gimbal_angles(out,angle_size,d)==0 && out[0]=='\0');
    assert(format_gimbal_angles(nullptr,sizeof(out),d)==0);
    assert(format_gimbal_angles(out,0,d)==0);
    d.yaw_encoder=3000; d.yaw_online=true;
    assert(format_commissioning_telemetry(out,sizeof(out),d)>0);
    assert(std::strcmp(out,"R_deg=0.00,Y_deg=0.00,gx_dps=0.00,gy_dps=0.00,gz_dps=0.00,IP_cmd=1000,IP_fb=950,IY_cmd=0,IY_fb=0,IMU=1\r\n")==0);
    assert(format_commissioning_telemetry(out,2,d)==0 && out[0]=='\0');
    const auto n=format_vofa_telemetry(out,sizeof(out),d);
    assert(n==std::strlen(out));
    assert(std::strcmp(out,"pitch:angle_deg=-70.25,target_deg=-72.50,I_cmd=1000,I_fb=950,encoder=5121,speed_dps=-0.50,state=1\r\n")==0);
    std::stringstream csv(std::string(out+6)); std::string token;
    std::vector<double> values;
    while(std::getline(csv,token,',')) values.push_back(std::stod(token.substr(token.find('=')+1)));
    assert(values.size()==7);
    assert(values[0]==-70.25 && values[1]==-72.5);
    assert(values[2]==1000 && values[3]==950 && values[4]==5121);
    assert(values[5]==-0.5 && values[6]==1);
    assert(format_vofa_telemetry(out,n+1,d)==n);
    assert(format_vofa_telemetry(out,n,d)==0 && out[0]=='\0');
    assert(format_vofa_telemetry(nullptr,sizeof(out),d)==0);
    assert(format_vofa_telemetry(out,0,d)==0);
    char tiny[2]; assert(format_vofa_telemetry(tiny,sizeof(tiny),d)==0 && tiny[0]=='\0');
    // 即便失去IMU/电机反馈也持续打印，显式标记-1，避免冻结假象。
    d.imu_fresh=false;
    assert(format_vofa_telemetry(out,sizeof(out),d)>0 && std::strstr(out,",state=-1\r\n"));
    d.imu_fresh=true; d.pitch_online=false;
    assert(format_vofa_telemetry(out,sizeof(out),d)>0 && std::strstr(out,",state=-1\r\n"));
    d.pitch_online=true; d.pitch_y_valid=false;
    assert(format_vofa_telemetry(out,sizeof(out),d)>0 && std::strstr(out,",state=-1\r\n"));
    d.pitch_y_valid=true; d.pitch_control_deg=std::numeric_limits<float>::quiet_NaN();
    assert(format_vofa_telemetry(out,sizeof(out),d)>0 && std::strstr(out,"pitch:angle_deg=0.00,") && std::strstr(out,",state=-1\r\n"));
    d.pitch_control_deg=-70.25f; d.pitch_hold_state=4; d.pitch_current_command=0;
    assert(format_vofa_telemetry(out,sizeof(out),d)>0 && std::strstr(out,",state=4\r\n"));
    // 不再输出或依赖四元数/偏置等诊断值。
    d.q[0]=std::numeric_limits<float>::quiet_NaN();
    assert(format_vofa_telemetry(out,sizeof(out),d)>0 && std::strstr(out,",state=4\r\n"));
}
