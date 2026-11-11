#include "DamiaoMitYawCommand.hpp"
#include <cassert>
#include <limits>

int main() {
    using namespace damiao_mit_yaw;
    CanFrame frame{};
    assert(make_special_frame(SpecialCommand::Enable, frame));
    assert(frame.standard_id == 0x06 && frame.data[7] == 0xfc);
    for (unsigned i = 0; i < 7; ++i) assert(frame.data[i] == 0xff);
    assert(make_special_frame(SpecialCommand::Disable, frame));
    assert(frame.data[7] == 0xfd);
    assert(!make_special_frame(static_cast<SpecialCommand>(0xfe), frame));
    assert(frame.data[7] == 0xfd);
    assert(make_control_frame({0, 0, 0, 0, 0}, frame));
    const uint8_t expected[] = {0x7f,0xff,0x7f,0xf0,0,0,0x07,0xff};
    assert(frame.standard_id == 0x06);
    for (unsigned i = 0; i < 8; ++i) assert(frame.data[i] == expected[i]);
    assert(!make_control_frame({0, 0, 0, 0, 11}, frame));
    assert(!make_control_frame({0, 0, 0, 0, std::numeric_limits<float>::quiet_NaN()}, frame));
    assert(!make_control_frame({12.5f, 30, 500, 5, 10}, frame));
    for (unsigned i = 0; i < 8; ++i) assert(frame.data[i] == expected[i]);
}
