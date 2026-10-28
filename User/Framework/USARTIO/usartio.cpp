#include "usartio.hpp"
#include "remotec.hpp"
#include "remote_struct.h"
#include "CRCCheck.h"
#include "cmath"
#include "debugc.hpp"


static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

static void vtm_to_rc(VT03_Receive_t *vtm_buf, VT03_ctrl_t *rc_ctrl);
static uint8_t* vtm_check_frame(uint8_t* StartByte);
static void vtm_unpack(uint8_t* buf_data);

static void portHandle(Key_State *port);
static void DataHandle(RC_ctrl_t *rc_ctrl);
static void DataHandle(VT03_ctrl_t *vtm_ctrl);
static void Key_PortHandle(RC_ctrl_t *rc_ctrl_);
static void Key_PortHandle(VT03_ctrl_t *vtm_ctrl);
static void key_reserved(RC_ctrl_t *rc_ctrl_);
static void key_reserved(VT03_ctrl_t *vtm_ctrl);

static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
static uint8_t vtm_rx_buf[2][VTM_RX_BUF_NUM];
static VT03_Receive_t vt03_prim_buf;

Class_Dr16 dr16(&huart3,DMA_IDLE_IT ,DR16);
Class_VT03 vt03(&huart6,DMA_IDLE_IT ,VT03);

void remote_init()
{
    dr16.open(sbus_rx_buf[0],sbus_rx_buf[1],SBUS_RX_BUF_NUM);
    vt03.open(vtm_rx_buf[0],vtm_rx_buf[1],VTM_RX_BUF_NUM);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart3)
    {
        dr16.rxCallback(&huart3, Size);
    }
    if (huart == &huart6)
    {
        vt03.rxCallback(&huart6, Size);
    }
    if (huart == &huart1)
    {
        debug.rxCallback(&huart1, Size);
    }
}

void Class_Usart::open(uint8_t *DstAddress, uint8_t *SecondMemAddress, uint32_t DataLength)
{
    which_uart_->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    which_uart_->RxEventType = HAL_UART_RXEVENT_IDLE;
    which_uart_->RxXferSize    = DataLength;
    SET_BIT(which_uart_->Instance->CR3,USART_CR3_DMAR);

    __HAL_UART_ENABLE_IT(which_uart_, UART_IT_IDLE);
    do{
        __HAL_DMA_DISABLE(which_uart_->hdmarx);
    }while(((DMA_Stream_TypeDef  *)which_uart_->hdmarx->Instance)->CR & DMA_SxCR_EN);
    ((DMA_Stream_TypeDef  *)which_uart_->hdmarx->Instance)->PAR = (uint32_t)&which_uart_->Instance->DR;
    ((DMA_Stream_TypeDef  *)which_uart_->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;
    ((DMA_Stream_TypeDef  *)which_uart_->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;
    ((DMA_Stream_TypeDef  *)which_uart_->hdmarx->Instance)->NDTR = DataLength;
    SET_BIT(((DMA_Stream_TypeDef  *)which_uart_->hdmarx->Instance)->CR, DMA_SxCR_DBM);

    __HAL_DMA_ENABLE(which_uart_->hdmarx);
}

void Class_Usart::rxCallback(UART_HandleTypeDef *huart,uint8_t size)
{
    const uint16_t rx_buf_len = (remote_type_ == VT03) ? VTM_RX_BUF_NUM : SBUS_RX_BUF_NUM;
    if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET)
    {
        __HAL_DMA_DISABLE(huart->hdmarx);
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
        __HAL_DMA_SET_COUNTER(huart->hdmarx, rx_buf_len);
        switch (remote_type_)
        {
            case DR16:
                if(size == RC_FRAME_LENGTH)
                {
                    sbus_to_rc(sbus_rx_buf[0],&dr16.value());
                }
                break;
            case VT03:
                    vtm_unpack(vtm_rx_buf[0]);
                    vtm_to_rc(&vt03_prim_buf,&vt03.value());
                break;
            default:
                break;
        }
    }else{
        __HAL_DMA_DISABLE(huart->hdmarx);
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
        __HAL_DMA_SET_COUNTER(huart->hdmarx, rx_buf_len);
        switch (remote_type_)
        {
        case DR16:
            if(size == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[1],&dr16.value());
            }
            break;
        case VT03:
            vtm_unpack(vtm_rx_buf[1]);
            vtm_to_rc(&vt03_prim_buf,&vt03.value());
            break;
        default:
            break;
        }
    }
    __HAL_DMA_ENABLE(huart->hdmarx);
}

RC_ctrl_t& Class_Dr16::value()
{
    return dr16_value;
}

VT03_ctrl_t& Class_VT03::value()
{
    return vt03_value;
}

static void vtm_unpack(uint8_t* buf_data)
{
    static uint8_t *vtm_pack = nullptr;
    vtm_pack = vtm_check_frame(buf_data);
    while (vtm_pack != nullptr)
    {
        memcpy(
               (char*)&vt03_prim_buf,
               (char*)&( (VT03FullFrame_t*) buf_data)->data,
               sizeof(VT03_Receive_t)
               ); // 不保留帧头和帧尾
        break; // 退出while
    }
}

static uint8_t* vtm_check_frame(uint8_t* StartByte)
{
    static uint8_t *Start_Flag_Byte = nullptr;//储存帧首地址位置
    static uint16_t CRC16_Result = 0; // 2字节校验

    /* vt03 */
    Start_Flag_Byte = (uint8_t*)strchr((const char*)StartByte,0xA9);//字符串内寻找第0xA5，返回其地址
    if(Start_Flag_Byte == nullptr)//未接收到开始帧返回nullptr
    {
        return nullptr;
    }
    // sof_2校验
    if (Start_Flag_Byte[1] == 0x53) // 或 (uint8_t)*(uint8_t*)&Start_Flag_Byte[0]
    {
        // 包尾校验
        CRC16_Result = GetCRC16CheckSum(Start_Flag_Byte,19,CRC16_INIT_NUM); // 校验长度19字节，不含包尾2字节CRC
        if(CRC16_Result == (uint16_t)*(uint16_t*)&Start_Flag_Byte[19])//CRC包尾校验
        {
            return Start_Flag_Byte;
        }return nullptr;
    }
    return nullptr;
}

void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    dr16.live_time = 200;
    dr16.online = true;

    key_reserved(rc_ctrl);

    rc_ctrl->rc.ch[0] = ((int16_t)sbus_buf[0] | ((int16_t)sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((int16_t)(sbus_buf[1] >> 3) | ((int16_t)sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((int16_t)(sbus_buf[2] >> 6) | ((int16_t)sbus_buf[3] << 2) |          //!< Channel 2
                         ((int16_t)sbus_buf[4] << 10)) &
                        0x07ff;
    rc_ctrl->rc.ch[3] = ((int16_t)(sbus_buf[4] >> 1) | ((int16_t)sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                                         //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                                    //!< Switch right
    rc_ctrl->mouse.x = ((int16_t)sbus_buf[6]) | ((int16_t)sbus_buf[7] << 8);                  //!< Mouse X axis
    rc_ctrl->mouse.y = ((int16_t)sbus_buf[8]) | ((int16_t)sbus_buf[9] << 8);                  //!< Mouse Y axis
    rc_ctrl->mouse.z = ((int16_t)sbus_buf[10]) | ((int16_t)sbus_buf[11] << 8);                //!< Mouse Z axis 滚轮
    rc_ctrl->mouse.press_l.Now_State = sbus_buf[12];                                          //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r.Now_State = sbus_buf[13];                                          //!< Mouse Right Is Press ?
    rc_ctrl->key.value =
        ((int16_t)sbus_buf[14]) | ((int16_t)sbus_buf[15] << 8); //!< KeyBoard value  W S A D Q E SHIFT CTRL
    rc_ctrl->key.W = (rc_ctrl->key.value & 0x01);
    rc_ctrl->key.S = (rc_ctrl->key.value & 0x02) >> 1;
    rc_ctrl->key.A = (rc_ctrl->key.value & 0x04) >> 2;
    rc_ctrl->key.D = (rc_ctrl->key.value & 0x08) >> 3;
    rc_ctrl->key.SHIFT.Now_State = (rc_ctrl->key.value & 0x10) >> 4;
    rc_ctrl->key.CONTRL.Now_State = (rc_ctrl->key.value & 0x20) >> 5;
    rc_ctrl->key.Q.Now_State = (rc_ctrl->key.value & 0x40) >> 6;
    rc_ctrl->key.E.Now_State = (rc_ctrl->key.value & 0x80) >> 7;
    rc_ctrl->key.R.Now_State = (rc_ctrl->key.value & 0x100) >> 8;
    rc_ctrl->key.F.Now_State = (rc_ctrl->key.value & 0x200) >> 9;
    rc_ctrl->key.G.Now_State = (rc_ctrl->key.value & 0x400) >> 10;
    rc_ctrl->key.Z.Now_State = (rc_ctrl->key.value & 0x800) >> 11;
    rc_ctrl->key.X.Now_State = (rc_ctrl->key.value & 0x1000) >> 12;
    rc_ctrl->key.C.Now_State = (rc_ctrl->key.value & 0x2000) >> 13;
    rc_ctrl->key.V.Now_State = (rc_ctrl->key.value & 0x4000) >> 14;
    rc_ctrl->key.B.Now_State = (rc_ctrl->key.value & 0x8000) >> 15;
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8); // 拨盘

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

    DataHandle(rc_ctrl);
    Key_PortHandle(rc_ctrl);
}

static void vtm_to_rc(VT03_Receive_t *vtm_links_buf, VT03_ctrl_t *vtm_ctrl_buf)
{
if (vtm_links_buf == nullptr || vtm_ctrl_buf == nullptr)
    {
        return;
    }
    vt03.live_time = 200;
    vt03.online = true;

    key_reserved(vtm_ctrl_buf);

    vtm_ctrl_buf->rc.ch[0] = vtm_links_buf->rc_all_data.ch_0;
    vtm_ctrl_buf->rc.ch[1] = vtm_links_buf->rc_all_data.ch_1;
    vtm_ctrl_buf->rc.ch[2] = vtm_links_buf->rc_all_data.ch_2;
    vtm_ctrl_buf->rc.ch[3] = vtm_links_buf->rc_all_data.ch_3;
    vtm_ctrl_buf->rc.wheel = vtm_links_buf->rc_all_data.wheel;
    vtm_ctrl_buf->rc.mode_sw = vtm_links_buf->rc_all_data.mode_sw;
    vtm_ctrl_buf->rc.pause.Now_State = vtm_links_buf->rc_all_data.pause;
    vtm_ctrl_buf->rc.fn_1.Now_State = vtm_links_buf->rc_all_data.fn_1;
    vtm_ctrl_buf->rc.fn_2.Now_State = vtm_links_buf->rc_all_data.fn_2;
    vtm_ctrl_buf->rc.trigger.Now_State = vtm_links_buf->rc_all_data.trigger;

    vtm_ctrl_buf->mouse.x = vtm_links_buf->keyboard_mouse_all_data.mouse_x;
    vtm_ctrl_buf->mouse.y = vtm_links_buf->keyboard_mouse_all_data.mouse_y;
    vtm_ctrl_buf->mouse.z = vtm_links_buf->keyboard_mouse_all_data.mouse_z;

    vtm_ctrl_buf->mouse.press_l.Now_State = vtm_links_buf->keyboard_mouse_all_data.mouse_left;
    vtm_ctrl_buf->mouse.press_r.Now_State = vtm_links_buf->keyboard_mouse_all_data.mouse_right;
    vtm_ctrl_buf->mouse.press_m.Now_State = vtm_links_buf->keyboard_mouse_all_data.mouse_middle;

    vtm_ctrl_buf->key.W = (vtm_links_buf->keyboard_mouse_all_data.key & 0x01);
    vtm_ctrl_buf->key.S = (vtm_links_buf->keyboard_mouse_all_data.key & 0x02) >> 1;
    vtm_ctrl_buf->key.A = (vtm_links_buf->keyboard_mouse_all_data.key & 0x04) >> 2;
    vtm_ctrl_buf->key.D = (vtm_links_buf->keyboard_mouse_all_data.key & 0x08) >> 3;
    vtm_ctrl_buf->key.SHIFT.Now_State = (vtm_links_buf->keyboard_mouse_all_data.key & 0x10) >> 4;
    vtm_ctrl_buf->key.CONTRL.Now_State = (vtm_links_buf->keyboard_mouse_all_data.key & 0x20) >> 5;
    vtm_ctrl_buf->key.Q.Now_State = (vtm_links_buf->keyboard_mouse_all_data.key & 0x40) >> 6;
    vtm_ctrl_buf->key.E.Now_State = (vtm_links_buf->keyboard_mouse_all_data.key & 0x80) >> 7;
    vtm_ctrl_buf->key.R.Now_State = (vtm_links_buf->keyboard_mouse_all_data.key & 0x100) >> 8;
    vtm_ctrl_buf->key.F.Now_State = (vtm_links_buf->keyboard_mouse_all_data.key & 0x200) >> 9;
    vtm_ctrl_buf->key.G.Now_State = (vtm_links_buf->keyboard_mouse_all_data.key & 0x400) >> 10;
    vtm_ctrl_buf->key.Z.Now_State = (vtm_links_buf->keyboard_mouse_all_data.key & 0x800) >> 11;
    vtm_ctrl_buf->key.X.Now_State = (vtm_links_buf->keyboard_mouse_all_data.key & 0x1000) >> 12;
    vtm_ctrl_buf->key.C.Now_State = (vtm_links_buf->keyboard_mouse_all_data.key & 0x2000) >> 13;
    vtm_ctrl_buf->key.V.Now_State = (vtm_links_buf->keyboard_mouse_all_data.key & 0x4000) >> 14;
    vtm_ctrl_buf->key.B.Now_State = (vtm_links_buf->keyboard_mouse_all_data.key & 0x8000) >> 15;

    vtm_ctrl_buf->rc.ch[0] -= VTM_CH_VALUE_OFFSET;
    vtm_ctrl_buf->rc.ch[1] -= VTM_CH_VALUE_OFFSET;
    vtm_ctrl_buf->rc.ch[2] -= VTM_CH_VALUE_OFFSET;
    vtm_ctrl_buf->rc.ch[3] -= VTM_CH_VALUE_OFFSET;
    vtm_ctrl_buf->rc.wheel -= VTM_CH_VALUE_OFFSET;

    Key_PortHandle(vtm_ctrl_buf);
    DataHandle(vtm_ctrl_buf);
}

static void portHandle(Key_State *port) {
    if (port->Now_State == 1 && port->Last_State == 0)
    {
        port->Is_Click_Once = 1;
    }
}

static void DataHandle(VT03_ctrl_t *vtm_ctrl)  //抑制零漂
{
    if (abs(vtm_ctrl->rc.ch[0]) < 5)vtm_ctrl->rc.ch[0] = 0;
    if (abs(vtm_ctrl->rc.ch[1]) < 5)vtm_ctrl->rc.ch[1] = 0;
    if (abs(vtm_ctrl->rc.ch[2]) < 5)vtm_ctrl->rc.ch[2] = 0;
    if (abs(vtm_ctrl->rc.ch[3]) < 5)vtm_ctrl->rc.ch[3] = 0;
    if (abs(vtm_ctrl->rc.wheel) < 5)vtm_ctrl->rc.wheel = 0;
    if (abs(vtm_ctrl->rc.ch[0]) > 700 || abs(vtm_ctrl->rc.ch[3]) > 700)
    {
        memset(&vtm_ctrl->rc, 0, sizeof(vtm_ctrl->rc)); //异常值,只清空遥控器数据，保留键鼠
    }
}

static void DataHandle(RC_ctrl_t *rc_ctrl)  //抑制零漂
{
    if (abs(rc_ctrl->rc.ch[0]) < 5)rc_ctrl->rc.ch[0] = 0;
    if (abs(rc_ctrl->rc.ch[1]) < 5)rc_ctrl->rc.ch[1] = 0;
    if (abs(rc_ctrl->rc.ch[2]) < 5)rc_ctrl->rc.ch[2] = 0;
    if (abs(rc_ctrl->rc.ch[3]) < 5)rc_ctrl->rc.ch[3] = 0;
    if (abs(rc_ctrl->rc.ch[4]) < 5)rc_ctrl->rc.ch[4] = 0;
    if (abs(rc_ctrl->rc.ch[0]) > 700 || abs(rc_ctrl->rc.ch[3]) > 700)
    {
        memset(&rc_ctrl->rc, 0, sizeof(rc_ctrl->rc)); //异常值,只清空遥控器数据，保留键鼠
    }
}

static void Key_PortHandle(RC_ctrl_t *rc_ctrl_) {
    portHandle(&rc_ctrl_->mouse.press_l);
    portHandle(&rc_ctrl_->mouse.press_r);
    portHandle(&rc_ctrl_->key.SHIFT);
    portHandle(&rc_ctrl_->key.CONTRL);
    portHandle(&rc_ctrl_->key.Q);
    portHandle(&rc_ctrl_->key.E);
    portHandle(&rc_ctrl_->key.R);
    portHandle(&rc_ctrl_->key.F);
    portHandle(&rc_ctrl_->key.G);
    portHandle(&rc_ctrl_->key.Z);
    portHandle(&rc_ctrl_->key.X);
    portHandle(&rc_ctrl_->key.C);
    portHandle(&rc_ctrl_->key.V);
    portHandle(&rc_ctrl_->key.B);
}

static void Key_PortHandle(VT03_ctrl_t *vtm_ctrl) {
    portHandle(&vtm_ctrl->rc.pause);
    portHandle(&vtm_ctrl->rc.fn_1);
    portHandle(&vtm_ctrl->rc.fn_2);
    portHandle(&vtm_ctrl->rc.trigger);
    portHandle(&vtm_ctrl->mouse.press_l);
    portHandle(&vtm_ctrl->mouse.press_r);
    portHandle(&vtm_ctrl->mouse.press_m);
    portHandle(&vtm_ctrl->key.SHIFT);
    portHandle(&vtm_ctrl->key.CONTRL);
    portHandle(&vtm_ctrl->key.Q);
    portHandle(&vtm_ctrl->key.E);
    portHandle(&vtm_ctrl->key.R);
    portHandle(&vtm_ctrl->key.F);
    portHandle(&vtm_ctrl->key.G);
    portHandle(&vtm_ctrl->key.Z);
    portHandle(&vtm_ctrl->key.X);
    portHandle(&vtm_ctrl->key.C);
    portHandle(&vtm_ctrl->key.V);
    portHandle(&vtm_ctrl->key.B);
}

void Key_Last_reserved(Key_State *port) {
    port->Last_State = port->Now_State;
}

void key_reserved(RC_ctrl_t *rc_ctrl_) {
    Key_Last_reserved(&rc_ctrl_->mouse.press_l);
    Key_Last_reserved(&rc_ctrl_->mouse.press_r);
    Key_Last_reserved(&rc_ctrl_->key.SHIFT);
    Key_Last_reserved(&rc_ctrl_->key.CONTRL);
    Key_Last_reserved(&rc_ctrl_->key.Q);
    Key_Last_reserved(&rc_ctrl_->key.E);
    Key_Last_reserved(&rc_ctrl_->key.R);
    Key_Last_reserved(&rc_ctrl_->key.F);
    Key_Last_reserved(&rc_ctrl_->key.G);
    Key_Last_reserved(&rc_ctrl_->key.Z);
    Key_Last_reserved(&rc_ctrl_->key.X);
    Key_Last_reserved(&rc_ctrl_->key.C);
    Key_Last_reserved(&rc_ctrl_->key.V);
    Key_Last_reserved(&rc_ctrl_->key.B);
}

void key_reserved(VT03_ctrl_t *vtm_ctrl) {
    Key_Last_reserved(&vtm_ctrl->rc.pause);
    Key_Last_reserved(&vtm_ctrl->rc.fn_1);
    Key_Last_reserved(&vtm_ctrl->rc.fn_2);
    Key_Last_reserved(&vtm_ctrl->rc.trigger);
    Key_Last_reserved(&vtm_ctrl->mouse.press_l);
    Key_Last_reserved(&vtm_ctrl->mouse.press_r);
    Key_Last_reserved(&vtm_ctrl->mouse.press_m);
    Key_Last_reserved(&vtm_ctrl->key.SHIFT);
    Key_Last_reserved(&vtm_ctrl->key.CONTRL);
    Key_Last_reserved(&vtm_ctrl->key.Q);
    Key_Last_reserved(&vtm_ctrl->key.E);
    Key_Last_reserved(&vtm_ctrl->key.R);
    Key_Last_reserved(&vtm_ctrl->key.F);
    Key_Last_reserved(&vtm_ctrl->key.G);
    Key_Last_reserved(&vtm_ctrl->key.Z);
    Key_Last_reserved(&vtm_ctrl->key.X);
    Key_Last_reserved(&vtm_ctrl->key.C);
    Key_Last_reserved(&vtm_ctrl->key.V);
    Key_Last_reserved(&vtm_ctrl->key.B);
}
