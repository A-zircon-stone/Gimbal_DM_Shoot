#pragma once
#include "PitchSpeedPidCommand.hpp"

inline uint8_t parse_yaw_gain(const char *line,float &value)
{
    return line && line[0]=='y' ? parse_pitch_speed_gain(line+1,value) : 0;
}
inline bool parse_yaw_delta(const char *line,float &value)
{
    if(!line || std::strncmp(line,"y:",2)) return false;
    char *end=nullptr;
    errno=0;
    const float parsed=std::strtof(line+2,&end);
    if(end==line+2 || errno==ERANGE || !std::isfinite(parsed)) return false;
    while(*end && std::isspace(static_cast<unsigned char>(*end))) ++end;
    if(*end) return false;
    value=parsed; return true;
}
