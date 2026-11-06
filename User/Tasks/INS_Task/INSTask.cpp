/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "INSTask.hpp"
#include "usart.h"
#include "PrintTelemetry.hpp"
#include "Gimbal.hpp"
#include "DamiaoPitchRuntime.hpp"
#include "DamiaoYawRuntime.hpp"
#define MPU6500_TEMP_PWM_MAX 5000
INS_t INS;
IMU_Param_t IMU_Param;
PID_s TempCtrl = {0};
static uint8_t first_temperate;
const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
static float RefTemp = 40;

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);

void INS_Init(void)
{
    IMU_Param.scale[X_] = 1;
    IMU_Param.scale[Y_] = 1;
    IMU_Param.scale[Z_] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;

    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);
    // imu heat init
    PID_Init(&TempCtrl, 2000, 300, 0, 1500, 20, 0, 0, 0, 0, 0, 0, 0);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

    INS.AccelLPF = 0.0085;
    // BMI088启动校准已耗时数秒，不能将这段时间作为第一帧积分dt。
    INS_DWT_Count = DWT->CYCCNT;
    // INS.AccelLPF = 1;

//    osDelay(50);
//    BMI088_Read(&BMI088);
//
//    RefTemp = BMI088.Temperature;
}

void INS_Task(void)
{
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};
    dt = DWT_GetDeltaT(&INS_DWT_Count);
    if (!isfinite(dt) || dt <= 0.0f || dt > 0.02f)
    {
        damiao_pitch_runtime::invalidate_imu();
        damiao_yaw_runtime::invalidate_imu();
        // 调试暂停/超期后丢弃这一帧，不用巨大dt推进姿态。
        ++count;
        return;
    }
    t += dt;

    // ins update
    if ((count % 1) == 0)
    {
        BMI088_Read(&BMI088);

        // ID 不匹配时驱动保留了上一次 Gyro，不能把它当成新的闭环反馈。
        // 此检查仅涵盖陀螺 ID，不代表所有 SPI 传输和加速度计数据有效。
        if (!BMI088_gyro_id_valid)
        {
            damiao_pitch_runtime::invalidate_imu();
            damiao_yaw_runtime::invalidate_imu();
            return;
        }

        INS.Accel[X_] = BMI088.Accel[X_];
        INS.Accel[Y_] = BMI088.Accel[Y_];
        INS.Accel[Z_] = BMI088.Accel[Z_];
        INS.Gyro[X_] = BMI088.Gyro[X_];
        INS.Gyro[Y_] = BMI088.Gyro[Y_];
        INS.Gyro[Z_] = BMI088.Gyro[Z_];

        // demo function,用于修正安装误差,可以不管,本demo暂时没用
        IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

        // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
        INS.atanxz = -atan2f(INS.Accel[X_], INS.Accel[Z_]) * 180 / PI;
        INS.atanyz = atan2f(INS.Accel[Y_], INS.Accel[Z_]) * 180 / PI;

        // 核心函数,EKF更新四元数
        IMU_QuaternionEKF_Update(INS.Gyro[X_], INS.Gyro[Y_], INS.Gyro[Z_], INS.Accel[X_], INS.Accel[Y_], INS.Accel[Z_], dt);
        if (!QEKF_INS.LastUpdateValid) {
            damiao_pitch_runtime::invalidate_imu();
            damiao_yaw_runtime::invalidate_imu();
            return;
        }

        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

        // 获取最终数据
        INS.Yaw = QEKF_INS.Yaw;
        INS.Pitch = QEKF_INS.Pitch;
        INS.Roll = QEKF_INS.Roll;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
        // Roll 和 Gyro[Y_] 同一帧一起发布，不直接跨任务读取更新中的 INS。
        damiao_pitch_runtime::publish_imu(INS.Roll, INS.Gyro[Y_], HAL_GetTick());
        damiao_yaw_runtime::publish_imu(INS.YawTotalAngle,INS.Roll,INS.Pitch,
                                        INS.Gyro[Y_],INS.Gyro[Z_],HAL_GetTick());
        Gimbal.Publish_Pitch_Feedback(INS.q, INS.Gyro[Y_]);
        publish_print_imu(INS.Roll, INS.Pitch, INS.Yaw, INS.Gyro, INS.q);
    }

    // temperature control
    if ((count % 2) == 0)
    {
        // 500hz
        imu_temp_control();
    }

    if ((count % 1000) == 0)
    {
        // 200hz
    }

    count++;
}


/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X_] = c_11 * gyro_temp[X_] +
               c_12 * gyro_temp[Y_] +
               c_13 * gyro_temp[Z_];
    gyro[Y_] = c_21 * gyro_temp[X_] +
               c_22 * gyro_temp[Y_] +
               c_23 * gyro_temp[Z_];
    gyro[Z_] = c_31 * gyro_temp[X_] +
               c_32 * gyro_temp[Y_] +
               c_33 * gyro_temp[Z_];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        accel_temp[i] = accel[i];

    accel[X_] = c_11 * accel_temp[X_] +
               c_12 * accel_temp[Y_] +
               c_13 * accel_temp[Z_];
    accel[Y_] = c_21 * accel_temp[X_] +
               c_22 * accel_temp[Y_] +
               c_23 * accel_temp[Z_];
    accel[Z_] = c_31 * accel_temp[X_] +
               c_32 * accel_temp[Y_] +
               c_33 * accel_temp[Z_];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

/**
 * @brief 温度控制
 *
 */
void IMU_Temperature_Ctrl(void)
{
    PID_Calculate(&TempCtrl, BMI088.Temperature, RefTemp);

    TIM_Set_PWM(&htim10, TIM_CHANNEL_1, fp32_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX));
}

void imu_temp_control(void)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        // controller.h: PID_Calculate(pid, measure, ref)，保持原PID及参数不变。
        PID_Calculate(&TempCtrl, BMI088.Temperature, RefTemp);
        if (TempCtrl.Output < 0.0f)
        {
            TempCtrl.Output = 0.0f;
        }
        tempPWM = (uint16_t)TempCtrl.Output;
        TIM_Set_PWM(&htim10, TIM_CHANNEL_1,tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (BMI088.Temperature > RefTemp)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                //达到设置温度，将积分项设置为一半最大功率，加速收敛
                //
                first_temperate = 1;
                TempCtrl.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
            }
        }

        TIM_Set_PWM(&htim10, TIM_CHANNEL_1,MPU6500_TEMP_PWM_MAX - 1);
    }
}
//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}

//------------------------------------2025.2.11-------------------------------------------------
static float IMU_AngleIncreLoop(float angle_now, float* last_angle, int32_t* rotate_times)
{
    float this_angle = angle_now;
    if ((this_angle - *last_angle) > 300)
        (*rotate_times)--;
    if ((this_angle - *last_angle) < -300)
        (*rotate_times)++;
    angle_now = this_angle + (*rotate_times) * 360.0f;
    *last_angle = this_angle;
    return angle_now;
}

float IMU_Angle(uint8_t which)
{
    static float last_pitch;
    static int32_t rotate_times_pitch = 0;

    switch (which)
    {
        case 1:
            return IMU_AngleIncreLoop(INS.Pitch, &last_pitch, &rotate_times_pitch);//pitch的角度为负
        case 2:
            return INS.YawTotalAngle;
        case 3:
            return INS.Roll; // Roll_angle直接返回，不需处理
        default:
            // 错误处理：返回0表示无效的请求
                return 0;
    }
}

float IMU_Speed(uint8_t which)
{
    switch (which)
    {
        case 1:
            return INS.Gyro[X_]*57.295779513f;//pitch的角度为负
        case 2:
            return INS.Gyro[Z_]*57.295779513f;
        case 3:
            return INS.Gyro[Y_]*57.295779513f;
        default:
            // 错误处理：返回0表示无效的请求
                return 0;
    }
}

void INSTask(void *argument)
{
    portTickType CurrentTime;
    osDelay(7);
    while (BMI088_init())
    {
    };

    INS_Init();
/* Infinite loop */
    for (;;) {
        CurrentTime = xTaskGetTickCount();
        INS_Task();
        INS.Wdg.clear_wdg();
        vTaskDelayUntil(&CurrentTime, 1 / portTICK_RATE_MS);
    }
}
