#include "remotec.hpp"
#include <cmath>
#include "usartio.hpp"
#include "Gimbal.hpp"



extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

eCtrlMode CtrlMode = eRC;
eAimMode AimMode = eManual;
eFricCtrlMode FricCtrlMode = eFric_SHUT;
eController Controller = eDR16;

/**********************************RC_Ctrl**********************************************/

/**
 * 保护检测 如果遥控器指定保护或遥控器断连
 * @return is_protect 1:保护 0:正常
 */
uint8_t portSetProtect(eRemote_type type)
{
    uint8_t is_protect;
    if (type == DR16)
    {
        dr16.live_time--;
        if (dr16.live_time <= 0)
        {
            is_protect = 1;
            dr16.online = false;
            dr16.value() = {};
        }
        else
        {
            is_protect = 0;
        }
        return is_protect;
    }
    if (type == VT03)
    {
        vt03.live_time--;
        if (vt03.live_time <= 0)
        {
            is_protect = 1;
            vt03.online = false;
            vt03.value() = {};
        }
        else
        {
            is_protect = 0;
        }
        return is_protect;
    }
    return 1;
}

/**
 * 遥控器仲裁：决定当前使用 DR16 还是 VT03
 */
void rc_arbitration()
{
    static size_t nonrc_tick = 0;
    static size_t vt03_hostflag = 0;

    portSetProtect(DR16);
    portSetProtect(VT03);
    switch (nonrc_tick)
    {
    case 2048:
        if(dr16.online == true)
        {
            nonrc_tick = 0;
            Controller = eDR16;
            vt03_hostflag = 0;
        }
        else if(vt03.online == true)
        {
            nonrc_tick = 0;
            Controller = eVT03;
            vt03_hostflag = 1;
        }
        else
            Controller = eNone;
        break;
    default:
        if (dr16.online == true)
        {
            Controller = eDR16;
            vt03_hostflag = 0;
            nonrc_tick = 0;
        }
        else if (vt03.online == true)
        {
            Controller = eVT03;
            vt03_hostflag = 1;
            nonrc_tick = 0;
        }
        else
        {
            nonrc_tick++;
        }
        break;
    }
    if(nonrc_tick > 10)
    {
        Controller = eNone;
        nonrc_tick = 2048;
    }
}

/**
 * 遥控器拨杆/按键模式解析
 * 负责切换 CtrlMode, AimMode, FricCtrlMode 等状态
 * TODO: 在此处添加你自己的底盘/云台模式切换逻辑
 */
void RC_Levers()
{
    if (Controller == eDR16)//DR16遥控器下
    {

        // S1=右拨杆s[1]：下档为手动发射，仍由摇杆控制。
        CtrlMode = eRC;

    }
    else if (Controller == eVT03)
    {
        if (vt03.value().rc.mode_sw == 0) CtrlMode = eKey;
        else CtrlMode = eRC;
    }
}

/**
 * 获取前后方向速度 (Vy)
 * @return 归一化到 [-100, 100] 的目标速度
 */
float portSetVy(void)
{
    static float target_vx = 0;

    if((Controller == eDR16 && dr16.online == false) 
    || (Controller == eVT03 && vt03.online == false) 
    || (Controller == eNone))
    {
        target_vx = 0;
        return target_vx;
    }

    if (Controller == eDR16)
    {
        switch (CtrlMode)
        {
        case eRC:
            target_vx = static_cast<float>(dr16.value().rc.ch[3]) * 100.0f / 660.0f;
            break;
        case eKey:
            target_vx = static_cast<float>(dr16.value().key.W - dr16.value().key.S) * 100.0f / 660.0f;
            break;
        default:
            target_vx = 0.0f;
            break;
        }
    }
    else if (Controller == eVT03)
    {
        switch (CtrlMode)
        {
        case eRC:
            target_vx = static_cast<float>(vt03.value().rc.ch[3]) * 100.0f / 660.0f;
            break;
        case eKey:
            target_vx = static_cast<float>(vt03.value().key.W - vt03.value().key.S) * 100.0f / 660.0f;
            break;
        default:
            target_vx = 0.0f;
            break;
        }
    }
    return target_vx;
}

/**
 * 获取左右方向速度 (Vx)
 * @return 归一化到 [-100, 100] 的目标速度
 */
float portSetVx(void)
{
    static float target_vy = 0;

    if((Controller == eDR16 && dr16.online == false) 
    || (Controller == eVT03 && vt03.online == false) 
    || (Controller == eNone))
    {
        target_vy = 0;
        return target_vy;
    }

    if(Controller == eDR16)
    {
        switch (CtrlMode)
        {
        case eRC:
            target_vy = static_cast<float>(dr16.value().rc.ch[2]) * 100.0f / 660.0f;
            break;
        case eKey:
            target_vy = static_cast<float>(dr16.value().key.A - dr16.value().key.D) * 100.0f / 660.0f;
            break;
        default:
            target_vy = 0.0f;
            break;
        }
    }
    else if (Controller == eVT03)
    {
        switch (CtrlMode)
        {
        case eRC:
            target_vy = static_cast<float>(vt03.value().rc.ch[2]) * 100.0f / 660.0f;
            break;
        case eKey:
            target_vy = static_cast<float>(vt03.value().key.A - vt03.value().key.D) * 100.0f / 660.0f;
            break;
        default:
            target_vy = 0.0f;
            break;
        }
    }

    return target_vy;
}

/**
 * 获取鼠标X轴输入（用于yaw控制）
 * @return 归一化鼠标X值
 */
float portGetMouseX(void)
{
    if((Controller == eDR16 && dr16.online == false)
    || (Controller == eVT03 && vt03.online == false) 
    || (Controller == eNone))
        return 0.0f;

    if (Controller == eDR16 && CtrlMode == eKey)
        return static_cast<float>(dr16.value().mouse.x);
    else if (Controller == eVT03 && CtrlMode == eKey)
        return static_cast<float>(vt03.value().mouse.x);
    return 0.0f;
}

/**
 * 获取鼠标Y轴输入（用于pitch控制）
 * @return 归一化鼠标Y值
 */
float portGetMouseY(void)
{
    if((Controller == eDR16 && dr16.online == false)
    || (Controller == eVT03 && vt03.online == false)
    || (Controller == eNone))
        return 0.0f;

    if (Controller == eDR16 && CtrlMode == eKey)
        return static_cast<float>(dr16.value().mouse.y);
    else if (Controller == eVT03 && CtrlMode == eKey)
        return static_cast<float>(vt03.value().mouse.y);
    return 0.0f;
}

/**
 * 获取右摇杆水平通道（ch[0]，通常用于yaw）
 * @return 归一化到 [-100, 100]
 */
float portGetCh0(void)
{
    if((Controller == eDR16 && dr16.online == false)
    || (Controller == eVT03 && vt03.online == false)
    || (Controller == eNone))
        return 0.0f;

    if (Controller == eDR16)
        return static_cast<float>(dr16.value().rc.ch[0]) * 100.0f / 660.0f;
    else if (Controller == eVT03)
        return static_cast<float>(vt03.value().rc.ch[0]) * 100.0f / 660.0f;
    return 0.0f;
}

/**
 * 获取右摇杆竖直通道（ch[1]，通常用于pitch）
 * @return 归一化到 [-100, 100]
 */
float portGetCh1(void)
{
    if((Controller == eDR16 && dr16.online == false)
    || (Controller == eVT03 && vt03.online == false) 
    || (Controller == eNone))
        return 0.0f;

    if (Controller == eDR16)
        return static_cast<float>(dr16.value().rc.ch[1]) * 100.0f / 660.0f;
    else if (Controller == eVT03)
        return static_cast<float>(vt03.value().rc.ch[1]) * 100.0f / 660.0f;
    return 0.0f;
}

/**
 * 获取拨轮通道（ch[4]）
 * @return 归一化到 [-100, 100]
 */
float portGetWheel(void)
{
    if((Controller == eDR16 && dr16.online == false)
    || (Controller == eVT03 && vt03.online == false) 
    || (Controller == eNone))
        return 0.0f;

    if (Controller == eDR16)
        return static_cast<float>(dr16.value().rc.ch[4]) * 100.0f / 660.0f;
    else if (Controller == eVT03)
        return static_cast<float>(vt03.value().rc.ch[4]) * 100.0f / 660.0f;
    return 0.0f;
}

/**
 * 获取鼠标左键状态
 */
uint8_t portGetMouseLeft(void)
{
    if (Controller == eDR16 && CtrlMode == eKey)
        return dr16.value().mouse.press_l.Now_State;
    else if (Controller == eVT03 && CtrlMode == eKey)
        return vt03.value().mouse.press_l.Now_State;
    return 0;
}

/**
 * 获取鼠标右键状态
 */
uint8_t portGetMouseRight(void)
{
    if (Controller == eDR16 && CtrlMode == eKey)
        return dr16.value().mouse.press_r.Now_State;
    else if (Controller == eVT03 && CtrlMode == eKey)
        return vt03.value().mouse.press_r.Now_State;
    return 0;
}

/**
 * 获取鼠标左键单击（toggle）
 */
uint8_t portGetMouseLeftClick(void)
{
    if (Controller == eDR16 && CtrlMode == eKey)
    {
        if (dr16.value().mouse.press_l.Is_Click_Once)
        {
            dr16.value().mouse.press_l.Is_Click_Once = 0;
            return 1;
        }
    }
    else if (Controller == eVT03 && CtrlMode == eKey)
    {
        if (vt03.value().mouse.press_l.Is_Click_Once)
        {
            vt03.value().mouse.press_l.Is_Click_Once = 0;
            return 1;
        }
    }
    return 0;
}

/**
 * 获取鼠标右键单击（toggle）
 */
uint8_t portGetMouseRightClick(void)
{
    if (Controller == eDR16 && CtrlMode == eKey)
    {
        if (dr16.value().mouse.press_r.Is_Click_Once)
        {
            dr16.value().mouse.press_r.Is_Click_Once = 0;
            return 1;
        }
    }
    else if (Controller == eVT03 && CtrlMode == eKey)
    {
        if (vt03.value().mouse.press_r.Is_Click_Once)
        {
            vt03.value().mouse.press_r.Is_Click_Once = 0;
            return 1;
        }
    }
    return 0;
}

/**
 * 获取鼠标Z轴（滚轮）
 */
int16_t portGetMouseZ(void)
{
    if((Controller == eDR16 && dr16.online == false)
    || (Controller == eVT03 && vt03.online == false)
    || (Controller == eNone))
        return 0;

    if (Controller == eDR16 && CtrlMode == eKey)
        return dr16.value().mouse.z;
    else if (Controller == eVT03 && CtrlMode == eKey)
        return vt03.value().mouse.z;
    return 0;
}

/* ==================== 键盘按键接口 ==================== */

// --- WASD 方向键（持续按住型，返回 0/1）---

uint8_t portGetKeyW(void)
{
    if (Controller == eDR16) return dr16.value().key.W;
    else if (Controller == eVT03) return vt03.value().key.W;
    return 0;
}

uint8_t portGetKeyS(void)
{
    if (Controller == eDR16) return dr16.value().key.S;
    else if (Controller == eVT03) return vt03.value().key.S;
    return 0;
}

uint8_t portGetKeyA(void)
{
    if (Controller == eDR16) return dr16.value().key.A;
    else if (Controller == eVT03) return vt03.value().key.A;
    return 0;
}

uint8_t portGetKeyD(void)
{
    if (Controller == eDR16) return dr16.value().key.D;
    else if (Controller == eVT03) return vt03.value().key.D;
    return 0;
}

// --- 功能键（Key_State型，提供 Now_State 持续状态 和 Is_Click_Once 切换状态）---

// SHIFT
uint8_t portGetKeyShift(void)
{
    if (Controller == eDR16) return dr16.value().key.SHIFT.Now_State;
    else if (Controller == eVT03) return vt03.value().key.SHIFT.Now_State;
    return 0;
}

// CTRL
uint8_t portGetKeyCtrl(void)
{
    if (Controller == eDR16) return dr16.value().key.CONTRL.Now_State;
    else if (Controller == eVT03) return vt03.value().key.CONTRL.Now_State;
    return 0;
}

// Q
uint8_t portGetKeyQ(void)
{
    if (Controller == eDR16) return dr16.value().key.Q.Now_State;
    else if (Controller == eVT03) return vt03.value().key.Q.Now_State;
    return 0;
}
uint8_t portGetKeyQClick(void)
{
    if (Controller == eDR16)
    {
        if (dr16.value().key.Q.Is_Click_Once)
        {
            dr16.value().key.Q.Is_Click_Once = 0;
            return 1;
        }
    }
    else if (Controller == eVT03)
    {
        if (vt03.value().key.Q.Is_Click_Once)
        {
            vt03.value().key.Q.Is_Click_Once = 0;
            return 1;
        }
    }
    return 0;
}

// E
uint8_t portGetKeyE(void)
{
    if (Controller == eDR16) return dr16.value().key.E.Now_State;
    else if (Controller == eVT03) return vt03.value().key.E.Now_State;
    return 0;
}
uint8_t portGetKeyEClick(void)
{
    if (Controller == eDR16)
    {
        if (dr16.value().key.E.Is_Click_Once)
        {
            dr16.value().key.E.Is_Click_Once = 0;
            return 1;
        }
    }
    else if (Controller == eVT03)
    {
        if (vt03.value().key.E.Is_Click_Once)
        {
            vt03.value().key.E.Is_Click_Once = 0;
            return 1;
        }
    }
    return 0;
}

// R
uint8_t portGetKeyR(void)
{
    if (Controller == eDR16) return dr16.value().key.R.Now_State;
    else if (Controller == eVT03) return vt03.value().key.R.Now_State;
    return 0;
}
uint8_t portGetKeyRClick(void)
{
    if (Controller == eDR16)
    {
        if (dr16.value().key.R.Is_Click_Once)
        {
            dr16.value().key.R.Is_Click_Once = 0;
            return 1;
        }
    }
    else if (Controller == eVT03)
    {
        if (vt03.value().key.R.Is_Click_Once)
        {
            vt03.value().key.R.Is_Click_Once = 0;
            return 1;
        }
    }
    return 0;
}

// F
uint8_t portGetKeyF(void)
{
    if (Controller == eDR16) return dr16.value().key.F.Now_State;
    else if (Controller == eVT03) return vt03.value().key.F.Now_State;
    return 0;
}
uint8_t portGetKeyFClick(void)
{
    if (Controller == eDR16)
    {
        if (dr16.value().key.F.Is_Click_Once)
        {
            dr16.value().key.F.Is_Click_Once = 0;
            return 1;
        }
    }
    else if (Controller == eVT03)
    {
        if (vt03.value().key.F.Is_Click_Once)
        {
            vt03.value().key.F.Is_Click_Once = 0;
            return 1;
        }
    }
    return 0;
}

// G
uint8_t portGetKeyG(void)
{
    if (Controller == eDR16) return dr16.value().key.G.Now_State;
    else if (Controller == eVT03) return vt03.value().key.G.Now_State;
    return 0;
}
uint8_t portGetKeyGClick(void)
{
    if (Controller == eDR16)
    {
        if (dr16.value().key.G.Is_Click_Once)
        {
            dr16.value().key.G.Is_Click_Once = 0;
            return 1;
        }
    }
    else if (Controller == eVT03)
    {
        if (vt03.value().key.G.Is_Click_Once)
        {
            vt03.value().key.G.Is_Click_Once = 0;
            return 1;
        }
    }
    return 0;
}

// Z
uint8_t portGetKeyZ(void)
{
    if (Controller == eDR16) return dr16.value().key.Z.Now_State;
    else if (Controller == eVT03) return vt03.value().key.Z.Now_State;
    return 0;
}
uint8_t portGetKeyZClick(void)
{
    if (Controller == eDR16)
    {
        if (dr16.value().key.Z.Is_Click_Once)
        {
            dr16.value().key.Z.Is_Click_Once = 0;
            return 1;
        }
    }
    else if (Controller == eVT03)
    {
        if (vt03.value().key.Z.Is_Click_Once)
        {
            vt03.value().key.Z.Is_Click_Once = 0;
            return 1;
        }
    }
    return 0;
}

// X
uint8_t portGetKeyX(void)
{
    if (Controller == eDR16) return dr16.value().key.X.Now_State;
    else if (Controller == eVT03) return vt03.value().key.X.Now_State;
    return 0;
}
uint8_t portGetKeyXClick(void)
{
    if (Controller == eDR16)
    {
        if (dr16.value().key.X.Is_Click_Once)
        {
            dr16.value().key.X.Is_Click_Once = 0;
            return 1;
        }
    }
    else if (Controller == eVT03)
    {
        if (vt03.value().key.X.Is_Click_Once)
        {
            vt03.value().key.X.Is_Click_Once = 0;
            return 1;
        }
    }
    return 0;
}

// C
uint8_t portGetKeyC(void)
{
    if (Controller == eDR16) return dr16.value().key.C.Now_State;
    else if (Controller == eVT03) return vt03.value().key.C.Now_State;
    return 0;
}
uint8_t portGetKeyCClick(void)
{
    if (Controller == eDR16)
    {
        if (dr16.value().key.C.Is_Click_Once)
        {
            dr16.value().key.C.Is_Click_Once = 0;
            return 1;
        }
    }
    else if (Controller == eVT03)
    {
        if (vt03.value().key.C.Is_Click_Once)
        {
            vt03.value().key.C.Is_Click_Once = 0;
            return 1;
        }
    }
    return 0;
}

// V
uint8_t portGetKeyV(void)
{
    if (Controller == eDR16) return dr16.value().key.V.Now_State;
    else if (Controller == eVT03) return vt03.value().key.V.Now_State;
    return 0;
}
uint8_t portGetKeyVClick(void)
{
    if (Controller == eDR16)
    {
        if (dr16.value().key.V.Is_Click_Once)
        {
            dr16.value().key.V.Is_Click_Once = 0;
            return 1;
        }
    }
    else if (Controller == eVT03)
    {
        if (vt03.value().key.V.Is_Click_Once)
        {
            vt03.value().key.V.Is_Click_Once = 0;
            return 1;
        }
    }
    return 0;
}

// B
uint8_t portGetKeyB(void)
{
    if (Controller == eDR16) return dr16.value().key.B.Now_State;
    else if (Controller == eVT03) return vt03.value().key.B.Now_State;
    return 0;
}
uint8_t portGetKeyBClick(void)
{
    if (Controller == eDR16)
    {
        if (dr16.value().key.B.Is_Click_Once)
        {
            dr16.value().key.B.Is_Click_Once = 0;
            return 1;
        }
    }
    else if (Controller == eVT03)
    {
        if (vt03.value().key.B.Is_Click_Once)
        {
            vt03.value().key.B.Is_Click_Once = 0;
            return 1;
        }
    }
    return 0;
}
