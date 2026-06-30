#include "motor_model.h"
#include <cstdio>
#include <array>
using namespace vsim;
static int step_delay_ms(MotorModel& m){
    // step duty 0 -> 0.5 at t=0, dt=1ms; find when omega crosses 5% of final
    std::array<float,4> d{0,0,0,0}; Vec3 F,T;
    for(int i=0;i<50;i++) m.update(d,0.001f,&F,&T);   // settle at 0
    d={0.5f,0.5f,0.5f,0.5f};
    float wfinal=0.5f*1200.0f; // target omega
    for(int k=0;k<1000;k++){ m.update(d,0.001f,&F,&T);
        if(m.omegas()[0] > 0.05f*wfinal) return k; }   // ms until 5% rise
    return -1;
}
int main(){
    {MotorParams p; MotorModel m; m.setParams(p); m.reset();
     printf("default (no delay):        omega 5%%-rise at %d ms (expect ~1-3)\n", step_delay_ms(m));}
    {MotorParams p; p.transport_delay=0.100f; MotorModel m; m.setParams(p); m.reset();
     printf("transport_delay=100ms:     omega 5%%-rise at %d ms (expect ~100-103)\n", step_delay_ms(m));}
    {MotorParams p; p.stall_duty=0.10f; MotorModel m; m.setParams(p); m.reset();
     // below stall_duty -> no thrust; step to 0.05 (<0.10) should give ZERO omega
     std::array<float,4> d{0.05f,0.05f,0.05f,0.05f}; Vec3 F,T;
     for(int i=0;i<200;i++) m.update(d,0.001f,&F,&T);
     printf("stall_duty=0.10, cmd 0.05: omega=%.1f thrust F.z=%.3f (expect ~0 = stalled)\n", m.omegas()[0], F.z());}
    return 0;
}
