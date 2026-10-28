#include "QuaternionEKF.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>

int main(int argc,char **argv)
{
    const int steps=argc>1?std::atoi(argv[1]):120000;
    IMU_QuaternionEKF_Init(10,.001f,10000000,1,0);
    float q[4]={-.603384f,.254217f,-.346238f,-.671879f};
    float n=0;for(float v:q)n+=v*v;n=std::sqrt(n);for(float &v:q)v/=n;
    if(argc<3)for(unsigned i=0;i<4;++i)QEKF_INS.IMU_QuaternionEKF.xhat_data[i]=q[i];
    const float ax=9.81f*2*(q[1]*q[3]-q[0]*q[2]);
    const float ay=9.81f*2*(q[0]*q[1]+q[2]*q[3]);
    const float az=9.81f*(q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]);
    for(int j=0;j<steps;++j){
        IMU_QuaternionEKF_Update(0,0,0,ax,ay,az,.001f);
        if(j==0||j==steps-1){float norm=0;for(float v:QEKF_INS.q)norm+=v*v;
            std::printf("step=%d yaw=%.6f R=%.6f P=%.6f norm2=%.9f bx=%.9f by=%.9f\n",j,QEKF_INS.Yaw,QEKF_INS.Roll,QEKF_INS.Pitch,norm,QEKF_INS.GyroBias[0],QEKF_INS.GyroBias[1]);}}
}
