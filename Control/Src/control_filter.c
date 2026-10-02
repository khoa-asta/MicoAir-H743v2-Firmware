#include "control_math.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

float Control_Clamp(float v,float lo,float hi) { return fminf(hi,fmaxf(lo,v)); }

/* Bilinear Butterworth low-pass, Q = 1/sqrt(2). */
bool ControlLpf_Init(ControlLpf_t *f,float cutoff,float fs)
{
    if(f==NULL || !isfinite(cutoff) || !isfinite(fs) || cutoff<=0 || fs<=0 || cutoff>=0.49f*fs) return false;
    memset(f,0,sizeof(*f));
    const float k=tanf(3.14159265358979323846f*cutoff/fs);
    const float n=1.0f/(1.0f+1.41421356237f*k+k*k);
    f->b0=k*k*n; f->b1=2.0f*f->b0; f->b2=f->b0;
    f->a1=2.0f*(k*k-1.0f)*n; f->a2=(1.0f-1.41421356237f*k+k*k)*n;
    return true;
}

/* Seed the filter at a constant input to avoid a startup transient. */
void ControlLpf_Reset(ControlLpf_t *f,float x)
{
    if(f==NULL) return;
    f->z1=(1.0f-f->b0)*x; f->z2=(f->b2-f->a2)*x; f->seeded=true;
}

bool ControlLpf_Update(ControlLpf_t *f,float x,float *y)
{
    if(f==NULL || y==NULL || !isfinite(x)) return false;
    if(!f->seeded) ControlLpf_Reset(f,x);
    const float value=f->b0*x+f->z1;
    const float z1=f->b1*x-f->a1*value+f->z2;
    const float z2=f->b2*x-f->a2*value;
    if(!isfinite(value)||!isfinite(z1)||!isfinite(z2)) return false;
    f->z1=z1; f->z2=z2; *y=value; return true;
}

/* Fixed-frequency biquad. No FFT/RPM tracking is implied by this interface. */
bool ControlNotch_Init(ControlLpf_t *f,float hz,float bw,float fs)
{
    if(f==NULL || !isfinite(hz) || !isfinite(bw) || !isfinite(fs) || fs<=0 ||
       hz<0 || hz>=0.49f*fs || bw<=0 || (hz>0 && bw>=hz)) return false;
    memset(f,0,sizeof(*f));
    if(hz==0) {f->b0=1;return true;}
    const float omega=6.28318530718f*hz/fs;
    const float alpha=sinf(omega)/(2.0f*(hz/bw));
    const float scale=1.0f/(1.0f+alpha);
    f->b0=scale;f->b1=-2.0f*cosf(omega)*scale;f->b2=scale;
    f->a1=f->b1;f->a2=(1.0f-alpha)*scale;
    return true;
}
