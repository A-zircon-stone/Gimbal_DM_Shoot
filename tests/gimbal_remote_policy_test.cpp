#include "GimbalRemotePolicy.hpp"
#include <cassert>
#include <initializer_list>

int main()
{
    using namespace gimbal_remote_policy;

    const auto protected_state = select(true, SwitchUp, SwitchUp);
    assert(protected_state.remote_mode);
    assert(protected_state.protected_mode);
    assert(!protected_state.autoaim_mode);

    const auto remote_state = select(true, SwitchMiddle, SwitchMiddle);
    assert(remote_state.remote_mode);
    assert(!remote_state.protected_mode);
    assert(!remote_state.autoaim_mode);

    for (uint8_t s2 : {SwitchUp, SwitchMiddle, SwitchDown}) {
        const auto manual_fire = select(true, SwitchDown, s2);
        assert(manual_fire.remote_mode);
        assert(!manual_fire.keyboard_mode);
        assert(!manual_fire.autoaim_mode);
        assert(manual_fire.manual_fire_mode);
    }

    const auto autoaim_state = select(true, SwitchMiddle, SwitchUp);
    assert(autoaim_state.remote_mode);
    assert(autoaim_state.autoaim_mode);

    const auto offline_autoaim = select(false, SwitchMiddle, SwitchUp);
    assert(offline_autoaim.remote_mode);
    assert(!offline_autoaim.autoaim_mode);
    assert(!select(false, SwitchDown, SwitchUp).manual_fire_mode);
    assert(!select(true, SwitchUp, SwitchDown).manual_fire_mode);
    assert(!select(true, SwitchMiddle, SwitchMiddle).manual_fire_mode);

    return 0;
}
