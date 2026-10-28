#include "QuaternionEKF.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <initializer_list>

static float clamp(float v){return v>1?1:(v< -1?-1:v);}
static void gravity(const float q[4], float a[3])
{
    a[0]=9.81f*2*(q[1]*q[3]-q[0]*q[2]);
    a[1]=9.81f*2*(q[0]*q[1]+q[2]*q[3]);
    a[2]=9.81f*(q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]);
}
static void update(float gx,float gy,float gz,const float a[3],float dt=.001f)
{ IMU_QuaternionEKF_Update(gx,gy,gz,a[0],a[1],a[2],dt); }
static void check_quaternion()
{
    float n=0;for(float v:QEKF_INS.q){assert(std::isfinite(v));n+=v*v;}
    assert(std::fabs(n-1)<1e-5f);
    assert(std::isfinite(QEKF_INS.Yaw)&&std::isfinite(QEKF_INS.Roll)&&std::isfinite(QEKF_INS.Pitch));
    assert(QEKF_INS.LastUpdateValid);
}
int main(int argc,char **argv)
{
    IMU_QuaternionEKF_Init(10,.001f,10000000,1,0);
    const char *mode=argc>1?argv[1]:"static";
    float a[3]={-7.4f,1.6f,6.2f};
    float n=std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    for(float &v:a)v*=9.81f/n;
    if(std::strcmp(mode,"cardinal")==0){
        const int axis=argc>2?argv[2][0]-'0':0;
        a[0]=a[1]=a[2]=0; a[axis%3]=(axis<3?9.81f:-9.81f);
    }
    update(0,0,0,a);check_quaternion();
    float expected[3];gravity(QEKF_INS.q,expected);
    for(unsigned i=0;i<3;++i)assert(std::fabs(expected[i]-a[i])<1e-4f);
    // 起始相对Yaw可任意选择；除极点不唯一情况之外应为0。
    if(std::strcmp(mode,"cardinal")!=0)assert(std::fabs(QEKF_INS.Yaw)<.001f);
    float initial[4];std::memcpy(initial,QEKF_INS.q,sizeof(initial));
    if(std::strcmp(mode,"invalid")==0){
        const auto count=QEKF_INS.UpdateCount;
        for(float dt:{0.f,-1.f,.021f,13.f,std::numeric_limits<float>::quiet_NaN()}){
            update(0,0,0,a,dt);assert(!QEKF_INS.LastUpdateValid);assert(QEKF_INS.UpdateCount==count);}
        const float zero[3]={};update(0,0,0,zero);assert(!QEKF_INS.LastUpdateValid);
        update(std::numeric_limits<float>::infinity(),0,0,a);assert(!QEKF_INS.LastUpdateValid);
        assert(std::memcmp(initial,QEKF_INS.q,sizeof(initial))==0);
        update(0,0,0,a);check_quaternion();
    }else if(std::strcmp(mode,"yaw")==0){
        // 保持倾斜，绕参考系竖直轴匀速转；机体系角速度为omega*g_b。
        const float rate=.2f;
        for(unsigned j=0;j<10000;++j){update(rate*a[0]/9.81f,rate*a[1]/9.81f,rate*a[2]/9.81f,a);check_quaternion();}
        const float target_yaw=rate*10*57.295779513f;
        assert(std::fabs(QEKF_INS.Yaw-target_yaw)<.2f);
        gravity(QEKF_INS.q,expected);
        for(unsigned i=0;i<3;++i)assert(std::fabs(expected[i]-a[i])<.02f);
    }else{
        for(unsigned j=0;j<60000;++j){update(0,0,0,a);check_quaternion();}
        float dot=0;for(unsigned i=0;i<4;++i)dot+=initial[i]*QEKF_INS.q[i];
        assert(2*std::acos(clamp(std::fabs(dot)))*57.295779513f<.1f);
        assert(std::fabs(QEKF_INS.GyroBias[0])<1e-5f&&std::fabs(QEKF_INS.GyroBias[1])<1e-5f);
    }
    std::printf("EKF %s passed: yaw=%.6f R=%.6f P=%.6f\n",mode,QEKF_INS.Yaw,QEKF_INS.Roll,QEKF_INS.Pitch);
}
