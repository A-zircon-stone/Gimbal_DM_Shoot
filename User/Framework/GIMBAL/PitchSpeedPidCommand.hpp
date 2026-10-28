#pragma once
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cstdint>

// USART1完整行：kp=/ki=/kd=修改速度环，kpp=修改位置环Kp。
inline uint8_t parse_pitch_speed_gain(const char *line, float &value)
{
    if (!line) return 0;
    uint8_t mask=0;
    unsigned prefix_length=3;
    if (!std::strncmp(line,"kp=",3)) mask=1;
    else if (!std::strncmp(line,"ki=",3)) mask=2;
    else if (!std::strncmp(line,"kd=",3)) mask=4;
    else if (!std::strncmp(line,"kpp=",4)) { mask=8; prefix_length=4; }
    else return 0;
    char *end=nullptr;
    errno=0;
    const float parsed=std::strtof(line+prefix_length,&end);
    if (end==line+prefix_length || errno==ERANGE || !std::isfinite(parsed)) return 0;
    while (*end && std::isspace(static_cast<unsigned char>(*end))) ++end;
    if (*end) return 0;
    value=parsed;
    return mask;
}
