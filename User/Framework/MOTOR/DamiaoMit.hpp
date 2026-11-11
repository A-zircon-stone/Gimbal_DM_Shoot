#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

// 达妙 MIT 模式的数据转换工具：只负责处理 CAN 数据区的 8 个字节。
// 它不负责收发 CAN、判断电机是否在线、使能/失能电机或运行云台控制。
// 目前尚未接入实际电机控制流程，使用前需核对电机型号、固件及参数。
namespace damiao_mit {
// 电机内部设置的编码范围上限（不是云台机械限位或安全输出限幅）。
// 分别对应 P_MAX、V_MAX、T_MAX；每台电机均应读取实际值后填写。
// 有符号量以 [-上限, +上限] 映射到无符号整数。
struct Limits {
    float position; // 位置编码范围上限，rad
    float velocity; // 速度编码范围上限，rad/s
    float torque;   // 扭矩编码范围上限，Nm
};

// 待发送的 MIT 控制量。Kp/Kd 为电机 MIT 模式的增益，不是旧 6020 PID 参数。
struct Command {
    float position; // 目标位置，rad
    float velocity; // 目标速度，rad/s
    float kp;       // 位置增益，协议范围 0..500
    float kd;       // 速度增益，协议范围 0..5
    float torque;   // 前馈扭矩，Nm
};

// 电机反馈的数据区解码结果；状态值的含义需按具体固件手册判断。
struct Feedback {
    uint8_t motor_id_low; // 数据区 D[0] 的低 4 位；不是完整 CAN ID
    uint8_t status;       // 数据区 D[0] 的高 4 位，解析后的状态/故障码
    float position, velocity, torque; // 反馈位置、速度、扭矩；单位同上
    uint8_t mos_temperature;   // T_MOS：驱动 MOS 的平均温度，℃
    uint8_t rotor_temperature; // T_Rotor：手册定义为电机内部线圈的平均温度，℃
};

// 对参数进行基本有效性检查：不能为 NaN、无穷大或非正数。
inline bool valid(const Limits &l) {
    return std::isfinite(l.position) && std::isfinite(l.velocity) &&
           std::isfinite(l.torque) && l.position > 0 && l.velocity > 0 && l.torque > 0;//有一个不对就返回0
}
inline bool in_range(float value, float low, float high) {
    return std::isfinite(value) && value >= low && value <= high;
}
// 将实际值线性映射到 bits 位无符号整数，转换为整数时截去小数部分。
// 例如 16 位整数的范围是 0..65535。调用方必须先检查参数和值的范围。
inline uint32_t encode(float value, float low, float high, unsigned bits) {
    const uint32_t encoded_max = (1u << bits) - 1u;
    //1u << bits 表示把无符号整数 1 向左移动 bits 位，相当于计算 \(2^{bits}\)
    return static_cast<uint32_t>((static_cast<double>(value) - low) * encoded_max /
                                 (static_cast<double>(high) - low));//映射
}
// 将反馈的整数映射回 [-max, +max]。量化会带来少量误差。
inline float decode(uint32_t value, float max, unsigned bits) {
    return static_cast<float>((static_cast<double>(value) * 2.0 * max /
                               ((1u << bits) - 1u)) - max);
}

// 把一组控制量打包为 8 字节。失败返回 false 且不修改 out；不会自动截断超限值。
// l 的范围必须与电机的设置一致；调用前还需另行检查机械限位和安全输出。
inline bool encode_command(const Limits &l, const Command &c, uint8_t *out, std::size_t length) {
    if (!out || length != 8 || !valid(l) ||
        !in_range(c.position, -l.position, l.position) ||
        !in_range(c.velocity, -l.velocity, l.velocity) ||
        !in_range(c.torque, -l.torque, l.torque) ||
        !in_range(c.kp, 0, 500) || !in_range(c.kd, 0, 5)) return false;
    // 位置 16 位；速度、Kp、Kd、扭矩各 12 位，共 64 位（8 字节）。
    const auto p = encode(c.position, -l.position, l.position, 16);
    const auto v = encode(c.velocity, -l.velocity, l.velocity, 12);
    const auto kp = encode(c.kp, 0, 500, 12);
    const auto kd = encode(c.kd, 0, 5, 12);
    const auto t = encode(c.torque, -l.torque, l.torque, 12);
    // 依次填入 p[15:0]、v[11:0]、kp[11:0]、kd[11:0]、t[11:0]。
    // 相邻的 12 位量会共用一个字节：左移/右移用于把各段放到对应位置。
    const uint8_t bytes[8] = {static_cast<uint8_t>(p >> 8), static_cast<uint8_t>(p),
        static_cast<uint8_t>(v >> 4), static_cast<uint8_t>((v << 4) | (kp >> 8)),
        static_cast<uint8_t>(kp), static_cast<uint8_t>(kd >> 4),
        static_cast<uint8_t>((kd << 4) | (t >> 8)), static_cast<uint8_t>(t)};
    for (unsigned i = 0; i < 8; ++i) out[i] = bytes[i];
    return true;
}

// 将 8 字节反馈解码成实际量；失败返回 false 且不改变 out。
// 本函数看不到 CAN 帧头，调用方必须检查总线、帧类型和反馈 ID，
// 并根据实际 Master ID 等配置把反馈分配给正确电机。
inline bool decode_feedback(const Limits &l, const uint8_t *data, std::size_t length, Feedback &out) {
    if (!data || length != 8 || !valid(l)) return false;
    Feedback next{};
    next.motor_id_low = data[0] & 0x0f; // 取 D[0] 的 bit 3..0
    next.status = data[0] >> 4;        // 取 D[0] 的 bit 7..4，再移到低 4 位
    // 反馈字节 1..2 是位置；字节 3..5 包含速度和扭矩，各占 12 位。
    const uint32_t p = (static_cast<uint32_t>(data[1]) << 8) | data[2];
    const uint32_t v = (static_cast<uint32_t>(data[3]) << 4) | (data[4] >> 4);
    const uint32_t t = (static_cast<uint32_t>(data[4] & 0x0f) << 8) | data[5];
    next.position = decode(p, l.position, 16);
    next.velocity = decode(v, l.velocity, 12);
    next.torque = decode(t, l.torque, 12);
    next.mos_temperature = data[6]; next.rotor_temperature = data[7];
    out = next;
    return true;
}
}
