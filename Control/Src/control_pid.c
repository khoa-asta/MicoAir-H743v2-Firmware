#include "control_math.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

bool ControlPid_Init(ControlPid_t *p,const ControlPidConfig_t *c)
{
    if(p==NULL||c==NULL) return false;
    const float v[]={c->kp,c->ki,c->kd,c->kff,c->i_limit,c->out_limit,c->aw_gain};
    for(unsigned i=0;i<sizeof(v)/sizeof(v[0]);i++) if(!isfinite(v[i])||v[i]<0) return false;
    if(c->out_limit<=0 || c->i_limit>c->out_limit) return false;
    memset(p,0,sizeof(*p)); p->config=*c; return true;
}

void ControlPid_Reset(ControlPid_t *p)
{
    if(p==NULL) return;
    p->integral=0; p->requested=0; p->p_term=0; p->d_term=0; p->have_request=false;
}

/* Bounded integration plus back-calculation from allocation/held actuator output. */
bool ControlPid_Update(ControlPid_t *p,float sp,float measured,float derivative,
                       float applied,float dt,bool integrate,float *out)
{
    if(p==NULL||out==NULL||!isfinite(sp)||!isfinite(measured)||!isfinite(derivative)||
       !isfinite(applied)||!isfinite(dt)||dt<=0||dt>0.05f) return false;
    const ControlPidConfig_t *c=&p->config;
    const float error=sp-measured;
    float integral=integrate?p->integral:0.0f;
    if(integrate)
    {
        const float unavailable=p->have_request?p->requested-applied:0.0f;
        /* Do not integrate farther into known actuator saturation. */
        const float error_term=(error*unavailable>1.0e-6f)?0.0f:c->ki*error;
        integral+=dt*(error_term-c->aw_gain*unavailable);
        integral=Control_Clamp(integral,-c->i_limit,c->i_limit);
    }
    p->p_term=c->kp*error;
    p->d_term=-c->kd*derivative;
    float request=p->p_term+integral+p->d_term+c->kff*sp;
    if(!isfinite(request)) return false;
    const float limited=Control_Clamp(request,-c->out_limit,c->out_limit);
    /* Next update compares this unlimited request to achieved/held output.
     * Do not apply a second back-calculation for the same saturation here. */
    p->integral=integral;
    p->requested=request; p->have_request=true;
    *out=limited; return true;
}
