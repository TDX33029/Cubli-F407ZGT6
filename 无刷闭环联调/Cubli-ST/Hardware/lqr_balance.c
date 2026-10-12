#include "lqr_balance.h"
#include "FOCMotor.h"
#include "BLDCMotor.h"
#include "timer.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
extern void set_motor_enable(uint8_t motor, uint8_t en);
extern uint8_t Control_Aborted(void);
extern void Control_RequestStop(uint8_t reason);
static uint32_t stall_since[3];
Cubli_Balance_t cubli_bal;
static float norm3(const float *v) {
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}
static float dot3(const float *a, const float *b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
static void cross3(const float *a, const float *b, float *r) {
    r[0]=a[1]*b[2]-a[2]*b[1]; r[1]=a[2]*b[0]-a[0]*b[2]; r[2]=a[0]*b[1]-a[1]*b[0];
}
static void clear_output(void) {
    int i;
    set_motor_enable(0, 0);
    for(i=0;i<3;i++) {
        cubli_bal.control_u[i]=cubli_bal.wheel_int[i]=0;
        voltage[i].q=voltage[i].d=0;
        stall_since[i]=0;
    }
}
void LQR_Trip(void) {
    clear_output(); cubli_bal.state=BAL_STATE_FALL_PROTECT;
}
void LQR_Balance_Init(void) {
    int i;
    memset(&cubli_bal,0,sizeof(cubli_bal));
    cubli_bal.max_angle_trip=15*DEG_TO_RAD;
    cubli_bal.capture_angle=5*DEG_TO_RAD;
    cubli_bal.K_yaw_damping=0.3f;
    for(i=0;i<3;i++) {
        LQR_Gains_t g={32.0f,2.4f,0.08f,0.008f,0,5};
        cubli_bal.gains[i]=cubli_bal.corner_gains[i]=g;
        cubli_bal.body_direction[i]=1;
    }
    cubli_bal.reference[0][2]=cubli_bal.reference[1][2]=1;
    cubli_bal.reference[2][0]=1;
    for(i=0;i<3;i++) cubli_bal.reference[3][i]=0.577350269f;
}
void LQR_Balance_Enable(bool enable) {
    int i,first,last;
    clear_output(); cubli_bal.state=BAL_STATE_DISABLED;
    if(!enable) return;
    if(!cubli_bal.attitude.valid || !cubli_bal.attitude.is_calibrated ||
       HAL_GetTick()-cubli_bal.attitude.last_sample_ms>30 || Control_Aborted()) return;
    first=cubli_bal.mode==BAL_MODE_CORNER?0:(int)cubli_bal.mode;
    last=cubli_bal.mode==BAL_MODE_CORNER?2:first;
    for(i=first;i<=last;i++) {
        updateSensor(i);
        if(!motor_aligned[i] || !sensor_valid[i]) return;
    }
    cubli_bal.last_update_us=micros();
    cubli_bal.state=BAL_STATE_STANDBY;
}
void LQR_Balance_StartCalib(void) {
    clear_output(); cubli_bal.state=BAL_STATE_CALIBRATING;
    cubli_bal.attitude.is_calibrated=false; cubli_bal.calib_samples=0;
    memset(cubli_bal.gyro_calib_sum,0,sizeof(cubli_bal.gyro_calib_sum));
}
void LQR_Attitude_Update(float a[3], float w[3], float dt) {
    int i;
    float n=norm3(a),pred[3],rot[3],alpha;
    Attitude_t *att=&cubli_bal.attitude;
    for(i=0;i<3;i++) if(!isfinite(a[i]) || !isfinite(w[i])) {att->valid=false;return;}
    if(n<0.2f || n>3.5f || dt<=0 || dt>0.03f) {att->valid=false;return;}
    if(cubli_bal.state==BAL_STATE_CALIBRATING) {
        if(fabsf(n-1)>0.15f || fabsf(w[0])>3 || fabsf(w[1])>3 || fabsf(w[2])>3) {
            cubli_bal.calib_samples=0;
            memset(cubli_bal.gyro_calib_sum,0,sizeof(cubli_bal.gyro_calib_sum)); return;
        }
        for(i=0;i<3;i++) cubli_bal.gyro_calib_sum[i]+=w[i]*DEG_TO_RAD;
        if(++cubli_bal.calib_samples==200) {
            for(i=0;i<3;i++) att->gyro_bias[i]=cubli_bal.gyro_calib_sum[i]/200;
            att->is_calibrated=true; att->valid=false;
            cubli_bal.state=BAL_STATE_DISABLED;
            printf("OK CAL_IMU:DONE\r\n");
        }
        return;
    }
    for(i=0;i<3;i++) att->gyro_rad[i]=w[i]*DEG_TO_RAD-att->gyro_bias[i];
    /* Body gravity propagation: dg/dt = g cross omega. No Euler singularity. */
    cross3(att->gravity,att->gyro_rad,rot);
    alpha=dt/(0.25f+dt);
    if(fabsf(n-1)>0.25f) alpha=0;
    for(i=0;i<3;i++) {
        pred[i]=att->gravity[i]+rot[i]*dt;
        pred[i]=(1-alpha)*pred[i]+alpha*a[i]/n;
        if(!att->valid) pred[i]=a[i]/n;
        att->accel_filt[i]=a[i];
    }
    n=norm3(pred);
    for(i=0;i<3;i++) att->gravity[i]=pred[i]/n;
    att->pitch=atan2f(att->gravity[1],sqrtf(att->gravity[0]*att->gravity[0]+att->gravity[2]*att->gravity[2]));
    att->roll=atan2f(-att->gravity[0],att->gravity[2]);
    att->yaw+=att->gyro_rad[2]*dt;
    att->pitch_deg=att->pitch*RAD_TO_DEG; att->roll_deg=att->roll*RAD_TO_DEG;
    att->valid=true; att->last_sample_ms=HAL_GetTick();
}
bool LQR_CaptureReference(void) {
    int i,m=(int)cubli_bal.mode;
    if(cubli_bal.state!=BAL_STATE_DISABLED || !cubli_bal.attitude.valid ||
       HAL_GetTick()-cubli_bal.attitude.last_sample_ms>30) return false;
    if(m<3 && fabsf(cubli_bal.attitude.gravity[m])>0.95f) return false;
    for(i=0;i<3;i++) cubli_bal.reference[m][i]=cubli_bal.attitude.gravity[i];
    if(m<3) cubli_bal.gains[m].theta_0=0;
    else for(i=0;i<3;i++) cubli_bal.corner_gains[i].theta_0=0;
    return true;
}
void LQR_Balance_Loop(void) {
    int i,first,last,m=(int)cubli_bal.mode;
    uint32_t now=micros();
    float dt=(now-cubli_bal.last_update_us)*1e-6f,err[3]={0},tilt,rate_yaw;
    float *g=cubli_bal.attitude.gravity,*ref=cubli_bal.reference[m];
    cubli_bal.last_update_us=now;
    bool active=cubli_bal.state==BAL_STATE_STANDBY || cubli_bal.state==BAL_STATE_BALANCING;
    if(!cubli_bal.attitude.valid) {if(active) LQR_Trip();return;}
    cross3(g,ref,err);
    if(m<3) {
        float denom=dot3(g,ref)-g[m]*ref[m];
        if(1-ref[m]*ref[m]<0.05f || 1-g[m]*g[m]<0.05f) {if(active) LQR_Trip();return;}
        err[m]=atan2f(err[m],denom)-cubli_bal.gains[m].theta_0;
        tilt=fabsf(err[m]);
    } else {
        float s=norm3(err);
        tilt=atan2f(s,dot3(g,ref));
        if(s>1e-6f) for(i=0;i<3;i++) err[i]*=tilt/s;
    }
    cubli_bal.tilt_deg=tilt*RAD_TO_DEG;
    if(!active) return;
    if(dt<=0 || dt>0.03f || HAL_GetTick()-cubli_bal.attitude.last_sample_ms>30 || Control_Aborted()) {LQR_Trip();return;}
    if(cubli_bal.state==BAL_STATE_BALANCING && tilt>cubli_bal.max_angle_trip) {LQR_Trip();return;}
    if(cubli_bal.state==BAL_STATE_STANDBY) {
        if(tilt>=cubli_bal.capture_angle) return;
        first=m<3?m:0; last=m<3?m:2;
        for(i=first;i<=last;i++) set_motor_enable(i+1,1);
        if(Control_Aborted()) {LQR_Trip();return;}
        cubli_bal.state=BAL_STATE_BALANCING;
    }
    rate_yaw=dot3(cubli_bal.attitude.gyro_rad,ref);
    first=m<3?m:0; last=m<3?m:2;
    for(i=first;i<=last;i++) {
        LQR_Gains_t *p=LQR_GetGains(i);
        float rate=cubli_bal.attitude.gyro_rad[i];
        float limit=fminf(p->max_u,voltage_limit),u,body;
        if(!sensor_valid[i] || !motor_aligned[i] || fabsf(shaft_velocity[i])>velocity_limit) {LQR_Trip();return;}
        if(m==3) rate-=rate_yaw*ref[i];
        body=p->K_theta*err[i]+p->K_dtheta*rate;
        if(m==3) body+=cubli_bal.K_yaw_damping*rate_yaw*ref[i];
        u=cubli_bal.body_direction[i]*body-p->K_w*shaft_velocity[i]-p->K_i*cubli_bal.wheel_int[i];
        if(fabsf(u)<limit) cubli_bal.wheel_int[i]=_constrain(cubli_bal.wheel_int[i]+shaft_velocity[i]*dt,-50,50);
        if(!isfinite(u)) {LQR_Trip();return;}
        cubli_bal.control_u[i]=_constrain(u,-limit,limit);
    }
}
void LQR_Motor_Loop(void) {
    int i,m=(int)cubli_bal.mode,first=m<3?m:0,last=m<3?m:2;
    if(cubli_bal.state!=BAL_STATE_BALANCING && cubli_bal.state!=BAL_STATE_STANDBY) return;
    if(Control_Aborted() || !cubli_bal.attitude.valid || HAL_GetTick()-cubli_bal.attitude.last_sample_ms>30) {LQR_Trip();return;}
    for(i=first;i<=last;i++) {
        loopFOC(i);
        if(!sensor_valid[i] || !motor_aligned[i]) {LQR_Trip();return;}
        if(cubli_bal.state==BAL_STATE_BALANCING) {
            if(fabsf(cubli_bal.control_u[i])>1.8f && fabsf(shaft_velocity[i])<0.1f) {
                uint32_t now=micros();
                if(!stall_since[i]) stall_since[i]=now;
                else if(now-stall_since[i]>800000U) {Control_RequestStop(7);LQR_Trip();return;}
            } else stall_since[i]=0;
            voltage[i].q=cubli_bal.control_u[i]; voltage[i].d=0;
            setPhaseVoltage(voltage[i].q,0,electrical_angle[i],i);
        }
    }
}
void LQR_Balance_SetMode(BalanceMode_e mode) {
    LQR_Balance_Enable(false);
    if(mode<=BAL_MODE_CORNER) cubli_bal.mode=mode;
}
LQR_Gains_t *LQR_GetGains(uint8_t idx) {
    if(idx>2) idx=0;
    return cubli_bal.mode==BAL_MODE_CORNER?&cubli_bal.corner_gains[idx]:&cubli_bal.gains[idx];
}
void LQR_SetGains(uint8_t idx,float kp,float kd,float kw,float ki) {
    LQR_Gains_t *g=LQR_GetGains(idx);
    g->K_theta=kp;g->K_dtheta=kd;g->K_w=kw;g->K_i=ki;
}
void LQR_SetZeroAngle(uint8_t idx,float deg) {LQR_GetGains(idx)->theta_0=deg*DEG_TO_RAD;}
void LQR_SetMaxVoltage(float v) {
    int i; for(i=0;i<3;i++) cubli_bal.gains[i].max_u=cubli_bal.corner_gains[i].max_u=v;
}
