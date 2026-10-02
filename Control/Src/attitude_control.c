#include "control_math.h"
#include <math.h>
#include <stddef.h>

bool AttitudeControl_Update(const Quaternion_t *q,const Quaternion_t *sp,const float kp[3],
                            const float limit[3],float yaw_rate_nav,float rates[3])
{
    Quaternion_t n,desired,inv,error;
    const float down[3]={0,0,1};
    float down_body[3];
    if(kp==NULL||limit==NULL||rates==NULL||!isfinite(yaw_rate_nav)||
       !Quaternion_Normalize(q,&n)||!Quaternion_Normalize(sp,&desired)||
       !Quaternion_Conjugate(&n,&inv)||!Quaternion_Multiply(&inv,&desired,&error)||
       !Quaternion_RotateVectorInverse(&n,down,down_body)) return false;
    const float sign=error.w<0?-1.0f:1.0f;
    const float vector[3]={error.x,error.y,error.z};
    for(int i=0;i<3;i++)
    {
        if(!isfinite(kp[i])||kp[i]<0||!isfinite(limit[i])||limit[i]<=0) return false;
        rates[i]=Control_Clamp(2.0f*sign*kp[i]*vector[i]+yaw_rate_nav*down_body[i],-limit[i],limit[i]);
    }
    return true;
}
