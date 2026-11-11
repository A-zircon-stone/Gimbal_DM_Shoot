#include "DamiaoMit.hpp"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

int main() {
    using namespace damiao_mit;
    // 以下值仅用于构造可计算的测试案例，不代表两台实物电机的设置。
    const Limits limits{12.5f, 30.f, 10.f};
    uint8_t out[8]{};
    // 检查各字段最低值、最高值以及零点对应的字节。
    assert(encode_command(limits, {-12.5f, -30.f, 0.f, 0.f, -10.f}, out, 8));
    for (auto b : out) assert(b == 0);
    assert(encode_command(limits, {12.5f, 30.f, 500.f, 5.f, 10.f}, out, 8));
    for (auto b : out) assert(b == 0xff);
    assert(encode_command(limits, {0.f, 0.f, 0.f, 0.f, 0.f}, out, 8));
    const uint8_t expected[] = {0x7f,0xff,0x7f,0xf0,0x00,0x00,0x07,0xff};
    for (unsigned i=0; i<8; ++i) assert(out[i] == expected[i]);
    // 超出范围、无效数字和无效配置必须失败，且不能改写上次输出。
    assert(!encode_command(limits, {0, 0, 0, 0, 11}, out, 8));
    assert(!encode_command(limits, {NAN, 0, 0, 0, 0}, out, 8));
    assert(!encode_command({0,30,10}, {0,0,0,0,0}, out, 8));
    for (unsigned i=0; i<8; ++i) assert(out[i] == expected[i]);
    // 构造一帧反馈，检查状态、位置、速度、扭矩和温度的解析。
    const uint8_t data[] = {0x81,0xff,0xff,0xff,0xff,0xff,42,37};
    Feedback f{};
    assert(decode_feedback(limits, data, 8, f));
    assert(f.status == 8 && f.motor_id_low == 1 && f.mos_temperature == 42 && f.rotor_temperature == 37);
    assert(f.position == 12.5f && f.velocity == 30.f && f.torque == 10.f);
    assert(!decode_feedback(limits, data, 7, f));
    assert(f.status == 8);
}
