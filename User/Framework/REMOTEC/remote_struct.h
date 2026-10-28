#ifndef INC_2026_HERO_GIMBAL_REMOTE_STRUCTION_H
#define INC_2026_HERO_GIMBAL_REMOTE_STRUCTION_H


typedef  struct __attribute__((packed)) {
    uint8_t Last_State: 1;
    uint8_t Now_State: 1;
    uint8_t Is_Click_Once: 1;       //“：1”声明后该变量只有0和1
} Key_State;

typedef struct __attribute__((packed)) {
    struct __attribute__((packed)){
        int16_t ch[5];  //无符号类型遥控器通道 0、1、2、3 控制信息 1683~1024~364
        // ch0->右摇杆横向   ch1->右摇杆纵向  ch2->左摇杆横向  ch3->左摇杆纵向
        char s[2];      //遥控发射机S1、S2开关  1:上 2:下 3:中
        // s[1]->s2(右边)    s[0]->s1(左边)
    } rc;
    struct __attribute__((packed)){
        int16_t x;      //鼠标值X轴 移动速度 32767 ~ -32768 静止值:0
        int16_t y;      //鼠标值Y轴
        int16_t z;      //鼠标值Z轴
//以下为使用到点按键值的键
        Key_State press_l;  //左键是否按下    0没按下 1按下
        Key_State press_r;  //右键是否按下
    } mouse;
    struct __attribute__((packed)){
        uint16_t value;
        uint8_t W: 1;
        uint8_t S: 1;
        uint8_t A: 1;
        uint8_t D: 1;
//以下为使用到点按键值的键
        Key_State SHIFT;
        Key_State CONTRL;
        Key_State Q;
        Key_State E;
        Key_State R;
        Key_State F;
        Key_State G;
        Key_State Z;
        Key_State X;
        Key_State C;
        Key_State V;
        Key_State B;
    } key;
} RC_ctrl_t;

/* ----------------------- VT13----------------------------------- */
typedef struct __attribute__((packed))
{
    uint8_t sof_1; // 0xA9
    uint8_t sof_2; // 0x53
} vt03_frame_header_t;

typedef struct __attribute__((packed))
{
    uint64_t ch_0:11;
    uint64_t ch_1:11;
    uint64_t ch_2:11;
    uint64_t ch_3:11;
    uint64_t mode_sw:2;
    uint64_t pause:1;
    uint64_t fn_1:1;
    uint64_t fn_2:1;
    uint64_t wheel:11;
    uint64_t trigger:1;
} vt03_rc_data_t;

typedef struct __attribute__((packed))
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint16_t key;
} vt03_keyboard_mouse_data_t;

typedef struct
{
    vt03_rc_data_t rc_all_data; // 遥控器数据
    vt03_keyboard_mouse_data_t keyboard_mouse_all_data; // 鼠标和键盘数据
} VT03_Receive_t;

typedef struct __attribute((packed))
{
    vt03_frame_header_t header;
    VT03_Receive_t data;
    uint16_t crc16;
} VT03FullFrame_t; // 一共21字节

typedef struct __attribute__((packed)) {
    struct __attribute__((packed)) {
        int16_t ch[4]; // 0:右水平 1:右竖直 2:左竖直 3:左水平
        int16_t wheel; // 拨盘
        uint8_t mode_sw; // 挡位切换开关 C:0 N:1 S:2
        Key_State pause; // 暂停键
        Key_State fn_1; // 左自定义按键
        Key_State fn_2; // 右自定义按键
        Key_State trigger; // 扳机
    } rc;
    struct __attribute__((packed)) {
        int16_t x;      //鼠标值X轴 移动速度 32767 ~ -32768 静止值:0
        int16_t y;      //鼠标值Y轴
        int16_t z;      //鼠标值Z轴
        //以下为使用到点按键值的键
        Key_State press_l;  //左键是否按下    0没按下 1按下
        Key_State press_r;  //右键是否按下
        Key_State press_m;
    } mouse;
    struct __attribute__((packed)) {
        uint16_t value;
        uint8_t W: 1;
        uint8_t S: 1;
        uint8_t A: 1;
        uint8_t D: 1;
        //以下为使用到点按键值的键
        Key_State SHIFT;
        Key_State CONTRL;
        Key_State Q;
        Key_State E;
        Key_State R;
        Key_State F;
        Key_State G;
        Key_State Z;
        Key_State X;
        Key_State C;
        Key_State V;
        Key_State B;
    } key;
} VT03_ctrl_t;


#endif //INC_2026_HERO_GIMBAL_REMOTE_STRUCTION_H