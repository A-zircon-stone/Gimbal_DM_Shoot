#include "DMMotor.hpp"
#include <cmath>
namespace damiao {
void Motor::Init(MotorConfig config)
{
    config_ = config;
    
    //谁调用 Init，函数就使用谁的成员变量。 每个电机对象都有自己的一份 config_。
}
void Motor::BuildEnableFrame(CanFrame& frame) 
{//enable motor
    frame.id = config_.motor_id;

    for (unsigned i = 0; i < 7; ++i) {
        frame.data[i] = 0xFF;
    }

    frame.data[7] = 0xFC;
}

void Motor::BuildDisableFrame(CanFrame& frame) 
{//disable motor
    frame.id = config_.motor_id;

    for (unsigned i = 0; i < 7; ++i) {
        frame.data[i] = 0xFF;
    }

    frame.data[7] = 0xFD;
}
// 将范围内的数值转换为协议整数编码
// 要求：max_value > min_value，value 在范围内，参数都是有限数值
uint16_t FloatToUint(float value,
                     float min_value,
                     float max_value,
                     uint16_t max_code)
{
    float ratio = (value - min_value) / (max_value - min_value);

    float code = ratio * max_code;

    return static_cast<uint16_t>(code);
}
bool Motor::BuildTorqueFrame(float torque, CanFrame& frame)
{
    // 1. 检查数值是否有效
    if (!std::isfinite(torque) ||
        !std::isfinite(config_.position_max) ||
        !std::isfinite(config_.velocity_max) ||
        !std::isfinite(config_.torque_max))
    {
        return false;
    }

    // 编码范围必须大于0，否则无法进行范围转换
    if (config_.position_max <= 0 ||
        config_.velocity_max <= 0 ||
        config_.torque_max <= 0)
    {
        return false;
    }

    // 超出电机协议能表示的范围，就不生成报文
    if (torque < -config_.torque_max ||
        torque > config_.torque_max)
    {
        return false;
    }

    // 2. 将位置、速度和力矩转换为整数编码
    uint16_t p = FloatToUint(
        0.0f, -config_.position_max, config_.position_max, 65535);

    uint16_t v = FloatToUint(
        0.0f, -config_.velocity_max, config_.velocity_max, 4095);

    uint16_t t = FloatToUint(
        torque, -config_.torque_max, config_.torque_max, 4095);

    // 3. 填写CAN报文，电机内部的Kp、Kd都设置为0
    frame.id = config_.motor_id;

    frame.data[0] = static_cast<uint8_t>(p >> 8);
    frame.data[1] = static_cast<uint8_t>(p & 0xFF);
    frame.data[2] = static_cast<uint8_t>(v >> 4);
    frame.data[3] = static_cast<uint8_t>((v & 0x0F) << 4);
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = static_cast<uint8_t>((t >> 8) & 0x0F);
    frame.data[7] = static_cast<uint8_t>(t & 0xFF);

    return true;
}
// 将协议整数编码还原为实际数值
// max_code必须大于0，value应在0到max_code之间
float UintToFloat(uint16_t value,
                  float min_value,
                  float max_value,
                  uint16_t max_code)
{
    float ratio = static_cast<float>(value) / max_code;

    float result = ratio * (max_value - min_value) + min_value;

    return result;
}
bool Motor::DecodeFeedback(
    uint16_t frame_id,
    const uint8_t* data,
    std::size_t length,
    MotorFeedback& feedback
) 
{
    // 1. 检查数据是否存在，长度是否为8字节
    if (data == nullptr || length != 8)
    {
        return false;
    }

    // 2. 检查反馈CAN ID
    if (frame_id != config_.master_id)
    {
        return false;
    }

    // 第0字节的低4位，是电机ID的低4位
    if ((data[0] & 0x0F) != (config_.motor_id & 0x0F))
    {
        return false;
    }

    // 3. 检查换算所需的编码范围
    if (!std::isfinite(config_.position_max) ||
        !std::isfinite(config_.velocity_max) ||
        !std::isfinite(config_.torque_max) ||
        config_.position_max <= 0 ||
        config_.velocity_max <= 0 ||
        config_.torque_max <= 0)
    {
        return false;
    }

    // 4. 从字节中拼出整数编码
    uint16_t p = (data[1] << 8) | data[2];

    uint16_t v = (data[3] << 4) | (data[4] >> 4);

    uint16_t t = ((data[4] & 0x0F) << 8) | data[5];

    // 5. 将整数编码还原成实际数值
    feedback.position = UintToFloat(
        p, -config_.position_max, config_.position_max, 65535);

    feedback.velocity = UintToFloat(
        v, -config_.velocity_max, config_.velocity_max, 4095);

    feedback.torque = UintToFloat(
        t, -config_.torque_max, config_.torque_max, 4095);

    // 6. 读取状态和温度
    feedback.status = data[0] >> 4;
    feedback.mos_temperature = data[6];
    feedback.rotor_temperature = data[7];

    return true;
}
} // namespace damiao