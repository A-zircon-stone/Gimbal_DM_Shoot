#include "PrintTelemetry.hpp"
#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>

int main()
{
    PrintTelemetry d{};
    d.time_ms = 1234;
    d.imu_sequence = 1000;
    d.imu_fresh = true;
    d.roll_deg = -12.25f;
    d.pitch_deg = 8.5f;
    d.yaw_deg = 179.0f;
    d.gyro_rad[0] = -1.5f;
    d.yaw_encoder = 8191;
    d.pitch_encoder = 123;
    d.yaw_rpm = -32768;
    d.pitch_rpm = 32767;
    d.yaw_online = true;
    d.pitch_y_deg = 105.25f;
    d.pitch_y_valid = true;
    char text[PrintTelemetryCapacity];
    assert(format_imu_attitude(text,sizeof(text),d)>0);
    assert(std::strcmp(text,"-12.2500,8.5000,179.0000\r\n")==0);
    assert(format_imu_attitude(text,2,d)==0 && text[0]=='\0');
    d.imu_fresh=false;
    assert(format_imu_attitude(text,sizeof(text),d)==0);
    d.imu_fresh=true;
    const auto size = format_print_telemetry(text, sizeof(text), d);
    assert(size > 0 && size == std::strlen(text));
    assert(std::strstr(text, "t=1234 N=1000 IMU=1 R=-12.25 P=+8.50 Y=+179.00"));
    assert(std::strstr(text, "gx=-1.500 gy=+0.000 gz=+0.000"));
    assert(std::strstr(text, "TY=+105.25 TV=1"));
    assert(std::strstr(text, "EY=8191 EP=123 rpmY=-32768 rpmP=32767 OY=1 OP=0\r\n"));
    char exact[PrintTelemetryCapacity];
    assert(format_print_telemetry(exact, size + 1, d) == size);
    assert(format_print_telemetry(exact, size, d) == 0 && exact[0] == '\0');
    char tiny[2] = {'x', 'y'};
    assert(format_print_telemetry(tiny, sizeof(tiny), d) == 0 && tiny[0] == '\0');
    assert(format_print_telemetry(text, 0, d) == 0);
    assert(format_print_telemetry(nullptr, 100, d) == 0);
    d.imu_fresh = false;
    assert(format_print_telemetry(text, sizeof(text), d) > 0);
    assert(std::strstr(text, "IMU=0"));
    assert(std::strstr(text, "TV=0"));
    d.imu_fresh = true;
    d.yaw_deg = std::numeric_limits<float>::quiet_NaN();
    assert(format_print_telemetry(text, sizeof(text), d) > 0);
    assert(std::strstr(text, "IMU=0"));
    d.yaw_deg = std::numeric_limits<float>::infinity();
    assert(format_print_telemetry(text, sizeof(text), d) > 0);
    assert(std::strstr(text, "IMU=0"));
    // 合法测量范围与最大计数下，一行仍放得下。
    d.time_ms = d.imu_sequence = 0xffffffffu;
    d.roll_deg = -180;
    d.pitch_deg = -90;
    d.yaw_deg = 180;
    d.gyro_rad[0] = d.gyro_rad[1] = d.gyro_rad[2] = -40;
    assert(format_print_telemetry(text, sizeof(text), d) > 0);
    assert(std::strstr(text, "IMU=1"));
    // 任意大float也不允许截断结果被当成有效帧发送。
    d.roll_deg = d.pitch_deg = d.yaw_deg = std::numeric_limits<float>::max();
    d.gyro_rad[0] = d.gyro_rad[1] = d.gyro_rad[2] = std::numeric_limits<float>::max();
    assert(format_print_telemetry(text, 320, d) == 0 && text[0] == '\0');
    std::cout << "Telemetry format tests passed\n";
}
