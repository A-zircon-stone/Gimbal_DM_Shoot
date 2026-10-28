#include "PrintTelemetry.hpp"
#include <cmath>
#include <cstdio>

std::size_t format_gimbal_angles(char *out,std::size_t capacity,const PrintTelemetry &d)
{
    if(!out || !capacity) return 0;
    const int size=std::snprintf(out,capacity,"angles:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
        static_cast<double>(d.yaw_control_deg),static_cast<double>(d.yaw_target_deg),
        static_cast<double>(d.yaw_target_deg-d.yaw_control_deg),
        static_cast<double>(d.pitch_control_deg),static_cast<double>(d.pitch_target_deg),
        static_cast<double>(d.pitch_target_deg-d.pitch_control_deg));
    if(size<0 || static_cast<std::size_t>(size)>=capacity)
    { out[0]='\0'; return 0; }
    return static_cast<std::size_t>(size);
}

std::size_t format_commissioning_telemetry(char *out,std::size_t capacity,
                                         const PrintTelemetry &d)
{
    if(!out || !capacity) return 0;
    const int size=std::snprintf(out,capacity,
        "R_deg=%.2f,Y_deg=%.2f,gx_dps=%.2f,gy_dps=%.2f,gz_dps=%.2f,IP_cmd=%d,IP_fb=%d,IY_cmd=%d,IY_fb=%d,IMU=%u\r\n",
        static_cast<double>(d.roll_deg),static_cast<double>(d.yaw_deg),
        static_cast<double>(d.gyro_rad[0]*57.295779513f),
        static_cast<double>(d.gyro_rad[1]*57.295779513f),
        static_cast<double>(d.gyro_rad[2]*57.295779513f),
        static_cast<int>(d.pitch_current_command),static_cast<int>(d.pitch_current_feedback),
        static_cast<int>(d.yaw_current_command),static_cast<int>(d.yaw_current_feedback),
        static_cast<unsigned>(d.imu_fresh));
    if(size<0 || static_cast<std::size_t>(size)>=capacity)
    { out[0]='\0'; return 0; }
    return static_cast<std::size_t>(size);
}

std::size_t format_vofa_telemetry(char *out, std::size_t capacity,
                                const PrintTelemetry &d)
{
    if (!out || capacity == 0) return 0;
    out[0]='\0';
    const bool finite=std::isfinite(d.pitch_control_deg) &&
        std::isfinite(d.pitch_target_deg) && std::isfinite(d.pitch_speed_deg_s);
    // 异常时也保留电流/状态输出，不让整行停更掩盖故障。-1表示反馈无效。
    const int state=d.imu_fresh && d.pitch_y_valid && d.pitch_online && finite ?
        static_cast<int>(d.pitch_hold_state) : -1;
    const int size=std::snprintf(out,capacity,"pitch:angle_deg=%.2f,target_deg=%.2f,I_cmd=%d,I_fb=%d,encoder=%u,speed_dps=%.2f,state=%d\r\n",
        static_cast<double>(std::isfinite(d.pitch_control_deg) ? d.pitch_control_deg : 0),
        static_cast<double>(std::isfinite(d.pitch_target_deg) ? d.pitch_target_deg : 0),
        static_cast<int>(d.pitch_current_command), static_cast<int>(d.pitch_current_feedback),
        static_cast<unsigned>(d.pitch_encoder),
        static_cast<double>(std::isfinite(d.pitch_speed_deg_s) ? d.pitch_speed_deg_s : 0),state);
    if(size<0 || static_cast<std::size_t>(size)>=capacity)
    { out[0]='\0'; return 0; }
    return static_cast<std::size_t>(size);
}

std::size_t format_print_telemetry(char *out, std::size_t capacity,
                                 const PrintTelemetry &d)
{
    if (!out || capacity == 0) return 0;
    const bool finite = std::isfinite(d.roll_deg) && std::isfinite(d.pitch_deg) &&
        std::isfinite(d.yaw_deg) && std::isfinite(d.gyro_rad[0]) &&
        std::isfinite(d.gyro_rad[1]) && std::isfinite(d.gyro_rad[2]);
    const int size = std::snprintf(out, capacity,
        "t=%lu N=%lu IMU=%u R=%+.2f P=%+.2f Y=%+.2f "
        "gx=%+.3f gy=%+.3f gz=%+.3f TY=%+.2f TV=%u EY=%u EP=%u "
        "rpmY=%d rpmP=%d OY=%u OP=%u\r\n",
        static_cast<unsigned long>(d.time_ms), static_cast<unsigned long>(d.imu_sequence),
        static_cast<unsigned>(d.imu_fresh && finite),
        static_cast<double>(d.roll_deg), static_cast<double>(d.pitch_deg),
        static_cast<double>(d.yaw_deg), static_cast<double>(d.gyro_rad[0]),
        static_cast<double>(d.gyro_rad[1]), static_cast<double>(d.gyro_rad[2]),
        static_cast<double>(d.pitch_y_deg),
        static_cast<unsigned>(d.imu_fresh && finite && d.pitch_y_valid && std::isfinite(d.pitch_y_deg)),
        static_cast<unsigned>(d.yaw_encoder), static_cast<unsigned>(d.pitch_encoder),
        static_cast<int>(d.yaw_rpm), static_cast<int>(d.pitch_rpm),
        static_cast<unsigned>(d.yaw_online), static_cast<unsigned>(d.pitch_online));
    if (size < 0 || static_cast<std::size_t>(size) >= capacity)
    {
        out[0] = '\0';
        return 0;
    }
    return static_cast<std::size_t>(size);
}

