/* Production control code with a PWM-to-encoder model at the hardware boundary. */
#include "mock_hw.h"
TIM_TypeDef mock_timers[3];GPIO_TypeDef mock_gpio;
static uint32_t fake_ms=100,fake_us=100000;
static const char *input_line;
static uint8_t encoder_ok=1,model_on=0;
static int16_t encoder_raw[3];
static int true_map[3]={2,0,1},true_sign[3]={1,-1,1},true_pp[3]={7,7,7};
static float rotor[3],field_previous[3];
static uint8_t field_seeded[3];
uint32_t HAL_GetTick(void) {return fake_ms;}
uint32_t micros(void) {return fake_us;}
void delay_ms(unsigned short ms) {fake_ms+=ms;fake_us+=ms*1000;Control_Heartbeat();}
void delay_us(unsigned long us) {fake_us+=(uint32_t)us;fake_ms=fake_us/1000;Control_Heartbeat();}
uint8_t USART_ReadLine(char *line) {if(!input_line)return 0;strcpy(line,input_line);input_line=NULL;return 1;}
void USART_ClearCommands(void) {}
uint16_t USART_TxFree(void) {return 1023;}
void UART_WriteText(const char *text) {fputs(text,stdout);}
void UART_WriteUInt(uint32_t value) {printf("%u",value);}
void UART_WriteInt(int32_t value) {printf("%d",value);}
void UART_WriteFixed(float value,uint8_t decimals) {printf("%.*f",decimals,value);}
uint16_t ENC_CountsPerRev(uint8_t idx) {return idx<3?16384:0;}
static void model(void) {
    int m;
    extern volatile uint8_t m1_enabled,m2_enabled,m3_enabled;
    uint8_t flags[3]={m1_enabled,m2_enabled,m3_enabled};
    if(!model_on) return;
    for(m=0;m<3;m++) if(flags[m]) {
        TIM_TypeDef *t=&mock_timers[m];
        float alpha=t->CCR1-0.5f*(t->CCR2+t->CCR3),beta=0.8660254f*((float)t->CCR2-t->CCR3);
        float field=atan2f(beta,alpha),delta;
        if(alpha*alpha+beta*beta<1) continue;
        if(!field_seeded[m]) {field_previous[m]=field;field_seeded[m]=1;}
        delta=field-field_previous[m];if(delta>3.14159265f)delta-=6.2831853f;if(delta< -3.14159265f)delta+=6.2831853f;
        rotor[m]+=delta*true_sign[m]/true_pp[m];field_previous[m]=field;
        {
            float wrapped=fmodf(rotor[m],6.2831853f);if(wrapped<0)wrapped+=6.2831853f;
            encoder_raw[true_map[m]]=(int16_t)(wrapped*16384/6.2831853f);
        }
    }
}
uint8_t ENC_GetAngle(uint8_t idx,int16_t *raw,float *deg) {
    model();*raw=encoder_raw[idx];*deg=*raw*360.0f/16384;return !encoder_ok;
}
uint8_t LSM6DSR_Init(void) {return 0;}
uint8_t MPU6050_Init(void) {return 0;}
#include "control_under_test.c"
#include "utils_under_test.c"
#include "foc_under_test.c"
#include "motor_under_test.c"
#include "balance_under_test.c"
static void command(const char *s) {input_line=s;commander_run();}
static void advance(void) {fake_ms+=5;fake_us+=5000;main_alive_ms=fake_ms;Control_Heartbeat();}
static void gravity(float x,float y,float z) {
 cubli_bal.attitude.gravity[0]=x;cubli_bal.attitude.gravity[1]=y;cubli_bal.attitude.gravity[2]=z;
 cubli_bal.attitude.valid=true;cubli_bal.attitude.last_sample_ms=fake_ms;
}
static void setup(void) {
 int i;fake_ms=fake_us/1000;model_on=0;encoder_ok=1;safety_trip=0;diagnostic_busy=0;
 mock_gpio.IDR=GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
 LQR_Balance_Init();SimpleFOC_PID_Init();voltage_power_supply=12;voltage_limit=5;velocity_limit=180;velocity_ramp_accel=400;pole_pairs=7;
 imu_online=1;Control_Heartbeat();main_alive_ms=fake_ms;controller=Type_velocity;
 command("APP 0");command("DBG 1");
 for(i=0;i<3;i++) {
  motor_pole_pairs[i]=7;motor_sensor_map[i]=(uint8_t)i;motor_mapping_valid[i]=1;motor_aligned[i]=sensor_valid[i]=1;
  sensor_direction[i]=1;encoder_raw[i]=0;rotor[i]=0;field_seeded[i]=0;
  reset_sensor_tracking(i);updateSensor(i);encoder_diag[i].last_us=fake_us;
 }
}
int main(void) {
 int i;float u;uint32_t saved;
 setvbuf(stdout,NULL,_IONBF,0);
 setup();command("DBG 2");command("EN1 1");command("EN2 1");command("EN3 1");assert(m1_enabled&&m2_enabled&&m3_enabled);
 command("M1 20");command("M2 -30");command("M3 40");assert(target_m1==20&&target_m2==-30&&target_m3==40);
 command("M 180 -180 0");assert(target_m1==180&&target_m2==-180&&target_m3==0);
 command("M2 nan");command("M2 8junk");assert(target_m2==-180);
 command("ACC 100");assert(velocity_ramp_accel==400);command("EN1 0");command("EN2 0");command("EN3 0");
 command("ACC 1000");assert(velocity_ramp_accel==1000);command("ACC 1501");assert(velocity_ramp_accel==1000);command("ACC 400");
 reset_motion_state(1);shaft_velocity_sp[1]=10;
 for(i=0;i<50;i++){fake_us+=1000;fake_ms++;ramp_velocity_target(-10,1);}
 assert(fabsf(shaft_velocity_sp[1]+10)<.001f);
 command("APP 1");assert(!m1_enabled&&!m2_enabled&&!m3_enabled&&target_m1==0&&target_m2==0);
 command("EN2 1");command("M2 20");assert(!m2_enabled&&target_m2==0);
 command("BAL 1");assert(cubli_bal.state==BAL_STATE_DISABLED);
 command("CAL_IMU");for(i=0;i<200;i++){float a[3]={0,0,1},w[3]={0};advance();LQR_Attitude_Update(a,w,.005f);}
 assert(cubli_bal.attitude.is_calibrated&&cubli_bal.state==BAL_STATE_DISABLED&&!m1_enabled);
 command("BAXIS 1 2 3");command("BDIR 1 1 1");command("BMODE 4");command("LQR 28 2.2 .07 .006");assert(LQR_GetGains(0)->K_theta==28);
 gravity(.577350269f,.577350269f,.577350269f);command("BAL 1");assert(cubli_bal.state==BAL_STATE_DISABLED);
 command("BZERO");command("BAL 1");LQR_Motor_Loop();advance();gravity(.577350269f,.577350269f,.577350269f);LQR_Balance_Loop();
 assert(cubli_bal.state==BAL_STATE_STANDBY&&!m1_enabled);
 for(i=0;i<21;i++){advance();gravity(.577350269f,.577350269f,.577350269f);LQR_Motor_Loop();LQR_Balance_Loop();}
 assert(cubli_bal.state==BAL_STATE_BALANCING&&m1_enabled&&m2_enabled&&m3_enabled);
 command("BAL 0");command("BMODE 1");gravity(0,0,1);command("BZERO");gravity(0,sinf(.02f),cosf(.02f));command("BAL 1");
 for(i=0;i<22;i++){advance();gravity(0,sinf(.02f),cosf(.02f));LQR_Motor_Loop();LQR_Balance_Loop();}
 assert(m1_enabled&&!m2_enabled&&!m3_enabled);u=cubli_bal.control_u[0];assert(u>0);
 sensor_direction[0]=-1;advance();gravity(0,sinf(.02f),cosf(.02f));LQR_Balance_Loop();assert(fabsf(cubli_bal.control_u[0]-u)<.001f);
 advance();gravity(0,sinf(.4f),cosf(.4f));LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_FALL_PROTECT&&!m1_enabled);
 advance();gravity(0,0,1);LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_FALL_PROTECT);
 /* Permute M1 onto the IMU Y axis: tilt about X must not drive M1. */
 command("BAL 0");command("BAXIS 2 3 1");command("BDIR 1 1 1");gravity(0,0,1);command("BZERO");command("BAL 1");
 for(i=0;i<22;i++){advance();gravity(sinf(-.02f),0,cosf(.02f));LQR_Motor_Loop();LQR_Balance_Loop();}
 assert(m1_enabled && cubli_bal.control_u[0]>0.5f);
 command("BAXIS 1 1 3");assert(cubli_bal.body_axis[0]==1);
 /* A single verified edge must not silently confirm the other wheel axes. */
 setup();command("BAXIS1 2");command("BDIR1 1");
 assert(cubli_bal.axes_confirmed_mask==1&&cubli_bal.directions_confirmed_mask==1);
 command("BCAP 11");assert(fabsf(cubli_bal.capture_angle-10*DEG_TO_RAD)<.0001f);
 command("BCAP 8");assert(fabsf(cubli_bal.capture_angle-8*DEG_TO_RAD)<.0001f);command("BCAP 10");
 command("BTRIP 26");assert(fabsf(cubli_bal.max_angle_trip-20*DEG_TO_RAD)<.0001f);
 command("BTRIP 18");assert(fabsf(cubli_bal.max_angle_trip-18*DEG_TO_RAD)<.0001f);command("BTRIP 20");
 assert(LQR_AxesConfirmed()&&LQR_DirectionsConfirmed());
 command("BMODE 4");assert(!LQR_AxesConfirmed()&&!LQR_DirectionsConfirmed());command("BMODE 1");
 command("APP 1");cubli_bal.attitude.is_calibrated=true;gravity(0,0,1);
 command("BREF 0 0 0");assert(!cubli_bal.reference_valid[0]);command("BREF 0 1 0");assert(!cubli_bal.reference_valid[0]);
 set_motor_enable(1,1);command("BREF 0 0 1");assert(!cubli_bal.reference_valid[0]);set_motor_enable(1,0);
 command("BSTART 3");assert(cubli_bal.state==BAL_STATE_DISABLED); /* Target must be captured deliberately. */
 command("BZERO");command("BREF 0 0 1");assert(cubli_bal.reference[0][2]==1);command("BSTART 3");assert(cubli_bal.state==BAL_STATE_WAIT_POSE&&!m1_enabled);
 for(i=0;i<21;i++){advance();gravity(0,0,1);LQR_Balance_Loop();assert(!m1_enabled);}
 assert(cubli_bal.state==BAL_STATE_COUNTDOWN);
 saved=cubli_bal.countdown_deadline_ms;command("BCAP 2");assert(cubli_bal.capture_angle==10*DEG_TO_RAD);
 cubli_bal.attitude.gyro_rad[1]=.4f;advance();gravity(0,0,1);LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_COUNTDOWN&&cubli_bal.countdown_deadline_ms==saved);cubli_bal.attitude.gyro_rad[1]=0;
 advance();gravity(-sinf(.2f),0,cosf(.2f));LQR_Balance_Loop();
 assert(cubli_bal.state==BAL_STATE_WAIT_POSE&&!m1_enabled&&!LQR_CountdownRemainingMs());
 assert(cubli_bal.reference[0][0]==0&&cubli_bal.reference[0][2]==1);
 for(i=0;i<21;i++){advance();gravity(0,0,1);LQR_Balance_Loop();assert(!m1_enabled);}
 assert(cubli_bal.state==BAL_STATE_COUNTDOWN);
 for(i=0;i<600;i++){advance();gravity(0,0,1);LQR_Balance_Loop();assert(!m1_enabled);}
 assert(cubli_bal.state==BAL_STATE_STANDBY&&cubli_bal.reference_valid[0]);
 for(i=0;i<22;i++){advance();gravity(0,0,1);LQR_Motor_Loop();LQR_Balance_Loop();}
 assert(cubli_bal.state==BAL_STATE_BALANCING&&m1_enabled&&!m2_enabled&&!m3_enabled);
 advance();gravity(-sinf(.3f),0,cosf(.3f));LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_BALANCING&&m1_enabled); /* 17.2deg is inside the requested 20deg limit. */
 /* Falling perpendicular to the controlled edge is still a real fall. */
 advance();gravity(0,sinf(.4f),cosf(.4f));LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_FALL_PROTECT&&!m1_enabled);
 for(i=0;i<30;i++){advance();gravity(0,0,1);LQR_Balance_Loop();}assert(cubli_bal.state==BAL_STATE_FALL_PROTECT);
 command("BSTART 3");assert(cubli_bal.state==BAL_STATE_WAIT_POSE);command("STOP");
 advance();gravity(0,0,1);LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_DISABLED&&!m1_enabled&&safety_trip==1);
 setup();command("APP 1");command("BAXIS1 2");command("BDIR1 1");cubli_bal.attitude.is_calibrated=true;gravity(0,0,1);
 command("BZERO");command("BSTART 3");cubli_bal.attitude.gyro_rad[1]=.2f;
 for(i=0;i<621;i++){advance();gravity(0,0,1);LQR_Balance_Loop();assert(!m1_enabled);}
 assert(cubli_bal.state==BAL_STATE_WAIT_POSE);cubli_bal.attitude.gyro_rad[1]=0;
 command("BSTART 3");fake_ms+=1501;Control_SafetyTick();assert(safety_trip==2&&!m1_enabled);
 /* Torque identification must time out on the MCU, without a host STOP. */
 setup();command("APP 1");cubli_bal.attitude.is_calibrated=true;gravity(0,0,1);
 command("BPROBE 0.75 80");assert(!m1_enabled);command("BAXIS1 2");
 assert(!cubli_bal.directions_confirmed_mask);command("BPROBE 3.1 80");assert(!m1_enabled);
 command("BPROBE 0.75 80");assert(cubli_bal.state==BAL_STATE_PROBE&&m1_enabled&&!m2_enabled&&!m3_enabled);
 LQR_Motor_Loop();assert(fabsf(voltage[0].q-.75f)<.0001f);
 fake_us+=79000;fake_ms+=79;gravity(0,0,1);LQR_Motor_Loop();assert(m1_enabled);
 fake_us+=1000;fake_ms++;gravity(0,0,1);LQR_Motor_Loop();assert(!m1_enabled&&cubli_bal.state==BAL_STATE_DISABLED&&voltage[0].q==0);
 command("BPROBE -0.75 80");LQR_Motor_Loop();assert(voltage[0].q<0);
 advance();gravity(-sinf(.3f),0,cosf(.3f));LQR_Balance_Loop();assert(!m1_enabled&&cubli_bal.state==BAL_STATE_FALL_PROTECT);
 setup();command("EN1 1");fake_ms+=1501;main_alive_ms=fake_ms;Control_SafetyTick();assert(safety_trip==2&&!m1_enabled);
 setup();command("EN1 1");mock_gpio.IDR&=~GPIO_PIN_9;Control_SafetyTick();assert(safety_trip==3&&!m1_enabled);
 setup();command("EN1 1");fake_us+=6000;fake_ms+=6;Control_Heartbeat();Control_SafetyTick();assert(safety_trip==8&&!m1_enabled);
 /* Arm a second axis after its idle sample has aged: acquire feedback first. */
 setup();command("EN1 1");fake_us+=80000;fake_ms+=80;Control_Heartbeat();main_alive_ms=fake_ms;updateSensor(0);
 saved=encoder_diag[2].samples;command("EN3 1");Control_SafetyTick();
 assert(m1_enabled&&m3_enabled&&!safety_trip&&encoder_diag[2].last_us==fake_us&&encoder_diag[2].samples>saved);
 /* Failed pre-arm feedback must leave every power stage disabled. */
 setup();command("EN1 1");encoder_ok=0;command("EN3 1");assert(safety_trip==6&&!m1_enabled&&!m3_enabled);
 setup();command("EN1 1");saved=encoder_diag[0].last_us;command("FREQ 1");assert(encoder_diag[0].last_us==saved);Control_SafetyTick();assert(!safety_trip);
 setup();command("EN1 1");encoder_ok=0;move(3,0);assert(safety_trip==6);
 setup();controller=Type_velocity_openloop;command("M1 20");assert(target_m1==0);motor_aligned[1]=0;command("EN2 1");assert(!m2_enabled);
 setup();controller=Type_velocity_openloop;voltage[0].q=5;shaft_velocity[0]=0;move(3,0);fake_us+=805000;fake_ms+=805;move(3,0);assert(safety_trip==7);
 /* Legitimate rotating saturation must NOT latch the previous 1.8V derating. */
 setup();controller=Type_velocity;shaft_velocity_sp[0]=180;
 for(i=0;i<1100;i++) {fake_us+=1000;fake_ms++;encoder_raw[0]=(encoder_raw[0]+26)%16384;move(180,0);}
 assert(!safety_trip && fabsf(voltage[0].q)>4.5f);
 setup();velocityOpenloop(0,0);assert(TIM1->CCR1==TIM1->CCR2&&TIM1->CCR2==TIM1->CCR3);
 setup();diagnostic_busy=1;model_on=1;memset(motor_mapping_valid,0,3);memset(motor_aligned,0,3);
 assert(Motor_alignSensor(0)==0 && motor_sensor_map[0]==2 && sensor_direction[0]==1);
 assert(Motor_alignSensor(1)==0 && motor_sensor_map[1]==0 && sensor_direction[1]==-1);
 assert(Motor_alignSensor(2)==0 && motor_sensor_map[2]==1 && sensor_direction[2]==1);
 assert(!m1_enabled&&!m2_enabled&&!m3_enabled);
 /* A different M2 pole count must fail alignment rather than silently pass. */
 setup();diagnostic_busy=1;model_on=1;memset(motor_mapping_valid,0,3);memset(motor_aligned,0,3);true_pp[1]=14;
 assert(Motor_alignSensor(1)!=0&&!motor_aligned[1]&&!m2_enabled);
 diagnostic_busy=0;command("PP2 14");assert(motor_pole_pairs[1]==14&&!motor_aligned[1]);
 diagnostic_busy=1;assert(Motor_alignSensor(1)==0&&motor_aligned[1]);true_pp[1]=7;
 setup();diagnostic_busy=1;model_on=0;assert(Motor_alignSensor(1)!=0&&!motor_aligned[1]&&!m2_enabled);
 puts("PASS: three-axis control, 50ms ramp, saturation, encoder freshness, balance latch, signed mapping and per-axis pole-pair sweep");
 return 0;
}
