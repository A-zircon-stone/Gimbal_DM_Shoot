//MIT 报文编码、反馈解码、使能/失能帧
#pragma once
//防止这个头文件在同一个编译单元中被重复展开，避免类、结构体重复定义。

#include <cstddef>
#include <cstdint>

namespace damiao {

// 标准 CAN 数据帧。
struct CanFrame {
    uint16_t id = 0;
    uint8_t data[8]{};
};
/*uint8_t   每个元素是一个8位无符号整数
data        数组名字
[8]         一共有8个元素
{}           将所有元素初始化为0*/

// 必须与电机上位机配置一致。
struct MotorConfig {
    uint16_t motor_id;
    uint16_t master_id;

    float position_max;  // MIT 编码范围，rad
    float velocity_max;  // MIT 编码范围，rad/s
    float torque_max;    // MIT 编码范围，N·m
};

struct MotorFeedback {
    uint8_t status = 0;

    float position = 0;
    float velocity = 0;
    float torque = 0;

    uint8_t mos_temperature = 0;
    uint8_t rotor_temperature = 0;
};

class Motor {
public:

    void BuildEnableFrame(CanFrame& frame) ;
    void BuildDisableFrame(CanFrame& frame);

    void Init(MotorConfig config);

    // MCU 已经算好力矩；电机 MIT Kp/Kd 保持为0。
    // 编码失败返回false，不修改frame。
    bool BuildTorqueFrame(float torque, CanFrame& frame);

    // 检查反馈ID、长度和电机身份。
    // 解码失败返回false，不修改feedback。
    bool DecodeFeedback(
        uint16_t frame_id,
        const uint8_t* data,
        std::size_t length,
        MotorFeedback& feedback
    ) ;

private:
    MotorConfig config_{};
};

} // namespace damiao