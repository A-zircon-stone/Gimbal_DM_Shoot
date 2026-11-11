#include "YawSoftLimit.hpp"
#include <cassert>
int main() {
    // 仅测试输入，非产品默认限位。角度使用连续值，不在正负 pi 处回绕。
    assert(yaw_soft_limit::apply(20.0f,false,0,0)==20.0f);
    assert(yaw_soft_limit::apply(13.0f,true,12.0f,14.0f)==13.0f);
    assert(yaw_soft_limit::apply(11.0f,true,12.0f,14.0f)==12.0f);
    assert(yaw_soft_limit::apply(15.0f,true,12.0f,14.0f)==14.0f);
    assert(yaw_soft_limit::apply(12.0f,true,12.0f,14.0f)==12.0f);
    assert(yaw_soft_limit::apply(14.0f,true,12.0f,14.0f)==14.0f);
}
