#ifndef REMOTEC_HPP
#define REMOTEC_HPP

#include "main.h"
#include "usart.h"
#include "usartio.hpp"
#include <cstring>

#define SBUS_RX_BUF_NUM 36u
#define VTM_RX_BUF_NUM 42u
#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

#define VTM_CH_VALUE_MIN         ((uint16_t)364)
#define VTM_CH_VALUE_OFFSET      ((uint16_t)1024)
#define VTM_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

typedef enum
{
    eRC,
    eKey
}   eCtrlMode; // 控制方式

typedef enum
{
    eManual = 0,
    eAutoaim
} eAimMode; // 瞄准方式


typedef enum
{
    eFric_SHUT = 0, // 关闭摩擦轮
    eFric_OPEN = 1, // 开启摩擦轮
} eFricCtrlMode;

typedef enum
{
    eAutoAim_OFF,
    eAutoAim_ON
} eAutoAimState;

typedef enum
{
    eDR16,
    eVT03,
    eNone
} eController;

extern eCtrlMode CtrlMode;
extern eAimMode AimMode;
extern eFricCtrlMode FricCtrlMode;
extern eController Controller;

#define TIMpiece 0.002f
#define LIMIT_VALUE(x, min, max)  x = (((x)<(min)) ? (min) : (((x)>(max)) ? (max) : (x)))

/* ---- 遥控器基础接口 ---- */
uint8_t portSetProtect(eRemote_type type);
void rc_arbitration(void);
void RC_Levers();

/* ---- 通道数据接口 ---- */
float portSetVy(void);                // 前后速度 (ch3/WS) → [-100,100]
float portSetVx(void);                // 左右速度 (ch2/AD) → [-100,100]
float portGetCh0(void);               // 右摇杆水平 (yaw方向) → [-100,100]
float portGetCh1(void);               // 右摇杆竖直 (pitch方向) → [-100,100]
float portGetWheel(void);             // 拨轮 → [-100,100]

/* ---- 鼠标接口 ---- */
float portGetMouseX(void);            // 鼠标X轴速度 (yaw)
float portGetMouseY(void);            // 鼠标Y轴速度 (pitch)
int16_t portGetMouseZ(void);          // 鼠标滚轮
uint8_t portGetMouseLeft(void);       // 鼠标左键 持续状态
uint8_t portGetMouseRight(void);      // 鼠标右键 持续状态
uint8_t portGetMouseLeftClick(void);  // 鼠标左键 单击切换
uint8_t portGetMouseRightClick(void); // 鼠标右键 单击切换

/* ---- 键盘接口 ---- */
// WASD (持续按住型，返回 0/1)
uint8_t portGetKeyW(void);
uint8_t portGetKeyS(void);
uint8_t portGetKeyA(void);
uint8_t portGetKeyD(void);
// 功能键 (Now_State: 持续状态)
uint8_t portGetKeyShift(void);
uint8_t portGetKeyCtrl(void);
uint8_t portGetKeyQ(void);
uint8_t portGetKeyE(void);
uint8_t portGetKeyR(void);
uint8_t portGetKeyF(void);
uint8_t portGetKeyG(void);
uint8_t portGetKeyZ(void);
uint8_t portGetKeyX(void);
uint8_t portGetKeyC(void);
uint8_t portGetKeyV(void);
uint8_t portGetKeyB(void);
// 功能键 (Is_Click_Once: 每按一次切换 0/1)
uint8_t portGetKeyQClick(void);
uint8_t portGetKeyEClick(void);
uint8_t portGetKeyRClick(void);
uint8_t portGetKeyFClick(void);
uint8_t portGetKeyGClick(void);
uint8_t portGetKeyZClick(void);
uint8_t portGetKeyXClick(void);
uint8_t portGetKeyCClick(void);
uint8_t portGetKeyVClick(void);
uint8_t portGetKeyBClick(void);

#endif //REMOTEC_HPP
