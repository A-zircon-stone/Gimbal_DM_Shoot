//
// Created by Lenovo on 2025/11/22.
//

#ifndef ZQL_INFANTRY_GIMBAL_PACKET_HPP
#define ZQL_INFANTRY_GIMBAL_PACKET_HPP


#include <algorithm>
#include <cstdint>
#include <vector>

struct VisionReceivePacket2026
{
    uint8_t head[2] = {'S', 'P'};    //帧头
    uint8_t mode = 0;     //0:不控制 1:控制不开火 （使能）
    uint8_t fire = 0;     //0:不开火 1:开火
    float yaw = 0.0f;
    float yaw_vel = 0.0f;
    float yaw_acc = 0.0f;
    float pitch = 0.0f;
    float pitch_vel = 0.0f;
    float pitch_acc = 0.0f;
    uint8_t shoot_count = 0;  // 打符 视觉期望射击次数
    uint16_t crc16 = 0;
} __attribute__((packed));

struct  VisionSendPacket2026
{
    uint8_t head[2] = {'S', 'P'};
    uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
    uint8_t enemy_color; // ‘R’ ‘B’
    float q[4];    // wxyz顺序
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;     //子弹的射速
    uint16_t bullet_count;  // 子弹累计发送次数
    uint8_t already_shoot_count; // 打符
    uint16_t crc16;
}__attribute__((packed));




//导航步兵未用
struct NavReceivePacket
{
    uint8_t header = 0xA6;
    float vel_x = 0.0f; // �����ٶȣ������Ҹ�
    float vel_y = 0.0f; // ǰ���ٶȣ�ǰ����
    float vel_z = 0.0f; // ��ת�ٶȣ����鷢0
    uint16_t checksum = 0;
} __attribute((packed));

struct NavSendPacket
{
    uint8_t header = 0x6A;
    float acc_x = 0.0f; // ������ٶȣ������Ҹ�
    float acc_y = 0.0f; // ǰ����ٶȣ�ǰ����
    float acc_z = 0.0f; // ��ֱ���ٶ�(һ����������������ٶ�)
    float gyro_yaw = 0.0f; // yaw����ٶ�
    float gyro_pitch = 0.0f; // pitch����ٶ�
    float gyro_roll = 0.0f; // roll����ٶȣ�û��
    float yaw = 0.0f; // yaw��Ƕ�
    float pitch = 0.0f; // pitch��Ƕ�
    float roll = 0.0f; // roll��Ƕȣ�û��
    float chassis_bias = 0.0f;// ��������̨�Ƕ�ƫ�������(��ƫ��ƫ)��Ҫ����
    uint16_t checksum = 0;
} __attribute__((packed));

extern VisionReceivePacket2026 vision_rv_pkt_2026;
extern VisionSendPacket2026 vision_sd_pkt_2026;
extern volatile uint32_t vision_rx_tick_ms;
extern volatile uint32_t vision_rx_sequence;


extern NavReceivePacket nav_rv_pkt;
extern NavSendPacket nav_sd_pkt;


//inline ReceivePacket fromVector(const uint8_t *data)
//{
//    ReceivePacket packet;
//    std::copy(data, data + sizeof(ReceivePacket), reinterpret_cast<uint8_t *>(&packet));
//    return packet;
//}

//inline uint8_t *toVector(const SendPacket &data)// & ?
//{
//    uint8_t packet[sizeof(SendPacket)];
//    std::copy(
//            reinterpret_cast<const uint8_t *>(&data),
//            reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet);
//    return packet;
//}

// ����ģ��
template <typename receivePacket>
inline receivePacket fromVector(const uint8_t *data)
{
//    static_assert(std::is_trivially_copyable_v<receivePacket>, "Target type must be trivially copyable for binary deserialization");

    receivePacket packet;
    std::copy(data, data + sizeof(receivePacket), reinterpret_cast<uint8_t *>(&packet));
    return packet;
}
/*// ģ���ػ�
template <>
inline VisionReceivePacket fromVector<VisionReceivePacket>(const uint8_t *data)
{
    VisionReceivePacket packet;
    std::copy(data, data + sizeof(VisionReceivePacket), reinterpret_cast<uint8_t *>(&packet));
    if (packet.header == 0x5A)
    {

    }
    return packet;
}*/

// todo ���ؿ�ָ�룡����
template <typename sendPacket>
inline uint8_t *toVector(const sendPacket &data)
{
    uint8_t packet[sizeof(sendPacket)];
    std::copy(
            reinterpret_cast<const uint8_t *>(&data),
            reinterpret_cast<const uint8_t *>(&data) + sizeof(sendPacket), packet);
    return packet;
}

#endif //ZQL_INFANTRY_GIMBAL_PACKET_HPP
