#include "Gm6020Feedback.hpp"
#include <cassert>
#include <iostream>

int main()
{
    uint8_t command[8];
    gm6020_encode_gimbal_current(600,-600,command);
    assert(command[0]==0x02 && command[1]==0x58);
    assert(command[2]==0 && command[3]==0); // ID2必须保持零
    assert(command[4]==0xfd && command[5]==0xa8); // Pitch ID3
    assert(command[6]==0 && command[7]==0);
    gm6020_encode_gimbal_current(0,0,command);
    for(auto byte:command) assert(byte==0);
    gm6020_encode_gimbal_current(0,3000,command);
    assert(command[0]==0 && command[1]==0 && command[2]==0 && command[3]==0);
    assert(command[4]==0x0b && command[5]==0xb8 && command[6]==0 && command[7]==0);
    Gm6020Feedback out{};
    const uint8_t bytes[8] = {0x1f, 0xff, 0xff, 0x9c, 0x80, 0, 45, 0};
    assert(gm6020_decode(0x205, 1, bytes, 8, out));
    assert(out.encoder == 8191 && out.speed_rpm == -100);
    assert(out.current_raw == -32768 && out.temperature_c == 45);
    assert(gm6020_decode(0x206, 2, bytes, 8, out));
    assert(gm6020_decode(0x207,GimbalHardware::PitchId,bytes,8,out));
    assert(!gm6020_decode(0x206,GimbalHardware::PitchId,bytes,8,out));
    assert(!gm6020_decode(0x205, 2, bytes, 8, out));
    assert(!gm6020_decode(0x205, 0, bytes, 8, out));
    assert(!gm6020_decode(0x20c, 8, bytes, 8, out));
    assert(!gm6020_decode(0x205, 1, nullptr, 8, out));
    assert(!gm6020_decode(0x205, 1, bytes, 7, out));
    assert(!gm6020_decode(0x205, 1, bytes, 9, out));
    const uint8_t bad_encoder[8] = {0x20, 0, 0, 0, 0, 0, 0, 0};
    assert(!gm6020_decode(0x205, 1, bad_encoder, 8, out));
    assert(out.encoder == 8191 && out.speed_rpm == -100);
    const uint8_t positive[8] = {0, 0, 0x7f, 0xff, 0xff, 0xff, 125, 0};
    assert(gm6020_decode(0x205, 1, positive, 8, out));
    assert(out.encoder == 0 && out.speed_rpm == 32767 && out.current_raw == -1);
    assert(!gm6020_feedback_fresh(false, 0, 0));
    assert(gm6020_feedback_fresh(true, 100, 0));
    assert(!gm6020_feedback_fresh(true, 101, 0));
    assert(gm6020_feedback_fresh(true, 0x20, 0xfffffff0u));
    assert(!gm6020_feedback_fresh(true, 0x100, 0xfffffff0u));
    static_assert(GimbalHardware::FeedbackOnly, "本阶段禁止使能旧控制链路");
    std::cout << "GM6020 feedback tests passed\n";
}
