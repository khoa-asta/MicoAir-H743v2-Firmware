#include "control_math.h"
#include <math.h>
#include <stddef.h>

/* Body FRD, rotor thrust -Z. CCW viewed from above gives positive yaw reaction.
 * Logical geometry order: FR CCW, RL CCW, FL CW, RR CW.
 * This is a configurable reference layout, not a verified wiring assertion. */
static const float coefficient[4][3]={{-1,1,1},{1,-1,1},{1,1,-1},{-1,-1,-1}};

void QuadMixer_Inverse(const float motor[4],float *collective,float effort[3])
{
    *collective=0;
    for(int j=0;j<3;j++) effort[j]=0;
    for(int i=0;i<4;i++)
    {
        *collective+=0.25f*motor[i];
        for(int j=0;j<3;j++) effort[j]+=0.25f*coefficient[i][j]*motor[i];
    }
}

/* Allocation priority: roll/pitch, then collective, then yaw. */
bool QuadMixer_Allocate(float t,const float u[3],float low,float high,QuadMixerOutput_t *o)
{
    if(u==NULL||o==NULL||!isfinite(t)||!isfinite(low)||!isfinite(high)||low<0||high>1||low>=high) return false;
    for(int j=0;j<3;j++) if(!isfinite(u[j])||fabsf(u[j])>10.0f) return false;
    float rp[4],minimum=INFINITY,maximum=-INFINITY;
    for(int i=0;i<4;i++)
    {
        rp[i]=coefficient[i][0]*u[0]+coefficient[i][1]*u[1];
        minimum=fminf(minimum,rp[i]); maximum=fmaxf(maximum,rp[i]);
    }
    const float span=maximum-minimum;
    o->rp_scale=span>high-low?(high-low)/span:1.0f;
    minimum*=o->rp_scale; maximum*=o->rp_scale;
    const float selected_t=Control_Clamp(t,low-minimum,high-maximum);
    o->yaw_scale=1.0f;
    for(int i=0;i<4;i++)
    {
        rp[i]*=o->rp_scale;
        const float base=selected_t+rp[i];
        const float yaw=coefficient[i][2]*u[2];
        if(yaw>0) o->yaw_scale=fminf(o->yaw_scale,(high-base)/yaw);
        else if(yaw<0) o->yaw_scale=fminf(o->yaw_scale,(low-base)/yaw);
    }
    o->yaw_scale=Control_Clamp(o->yaw_scale,0,1);
    for(int i=0;i<4;i++) o->motor[i]=Control_Clamp(selected_t+rp[i]+o->yaw_scale*coefficient[i][2]*u[2],low,high);
    QuadMixer_Inverse(o->motor,&o->collective,o->achieved);
    o->saturated=o->rp_scale<0.99999f||o->yaw_scale<0.99999f||fabsf(o->collective-t)>1.0e-5f;
    return true;
}
