//
// Created by Lenovo on 2025/11/22.
//

#ifndef ZQL_INFANTRY_GIMBAL_CRC_H
#define ZQL_INFANTRY_GIMBAL_CRC_H


#include <cstdint>


uint16_t Get_CRC16_Check_Sum(const uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(const uint8_t* pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);


#endif //ZQL_INFANTRY_GIMBAL_CRC_H
