#pragma once

namespace yaw_soft_limit {
// 上下限由配置保证有效；这里只约束目标，不因实际角越界而失能。
inline float apply(float target, bool enabled, float min_rad, float max_rad) {
    if (!enabled) return target;
    return target < min_rad ? min_rad : (target > max_rad ? max_rad : target);
}
}
