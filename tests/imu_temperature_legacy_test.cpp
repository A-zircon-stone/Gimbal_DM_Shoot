#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

// 只替代硬件/底层PID，下面被测试的两段函数直接从INSTask.cpp提取。
// 验证实参次序、冷热状态转换，不冒充真实热系统或PID整定测试。
#define MPU6500_TEMP_PWM_MAX 5000
static uint8_t first_temperate = 0;
static float RefTemp = 40;
struct { float Temperature = 0; } BMI088;
struct PID_s {float Output = 0, Iout = 0;} TempCtrl;
static float last_measure = 0, last_ref = 0;
static unsigned pid_calls = 0;
static uint16_t last_pwm = 0;
struct Timer {} htim10;
constexpr uint8_t TIM_CHANNEL_1 = 0;
void PID_Calculate(PID_s *pid, float measure, float ref)
{
    last_measure = measure;
    last_ref = ref;
    ++pid_calls;
    pid->Output = ref >= measure ? 100.0f : -100.0f;
}
float fp32_constrain(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
float float_rounding(float v){return std::round(v);}
void TIM_Set_PWM(Timer *,uint8_t,uint16_t v){last_pwm=v;}

#ifndef LEGACY_TEMPERATURE_SOURCE
#define LEGACY_TEMPERATURE_SOURCE "legacy_temperature.inc"
#endif
#include LEGACY_TEMPERATURE_SOURCE

#define CHECK(expr) do{if(!(expr)){std::fprintf(stderr,"failed: %s\n",#expr);return 1;}}while(false)
int main(int argc,char **argv)
{
    const char *mode = argc>1 ? argv[1] : "cold";
    if(std::strcmp(mode,"cold")==0)
    {
        BMI088.Temperature = 35;
        for(unsigned i=0;i<500;++i)imu_temp_control();
        CHECK(first_temperate==0);
        CHECK(pid_calls==0);
        CHECK(last_pwm==4999);
    }
    else if(std::strcmp(mode,"hot")==0)
    {
        BMI088.Temperature = 41;
        for(unsigned i=0;i<201;++i)imu_temp_control();
        CHECK(first_temperate==1);
        imu_temp_control();
        CHECK(last_measure==41 && last_ref==40);
        CHECK(last_pwm==0);
    }
    else
    {
        first_temperate=1;
        BMI088.Temperature=35;
        imu_temp_control();
        CHECK(last_measure==35 && last_ref==40 && last_pwm==100);
        BMI088.Temperature=42;
        imu_temp_control();
        CHECK(last_measure==42 && last_ref==40 && last_pwm==0);
        BMI088.Temperature=35;
        IMU_Temperature_Ctrl();
        CHECK(last_measure==35 && last_ref==40 && last_pwm==100);
    }
    std::printf("Legacy temperature %s passed\n",mode);
    return 0;
}
