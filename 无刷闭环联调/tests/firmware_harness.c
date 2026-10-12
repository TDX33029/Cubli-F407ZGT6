/* Executed with real commander, FOC, SVPWM and balance sources; only hardware is mocked. */
#include "mock_hw.h"
TIM_TypeDef mock_timers[3];
GPIO_TypeDef mock_gpio;
static uint32_t fake_ms=100,fake_us=100000;
static const char *input_line;
static uint8_t encoder_ok=1;
static int16_t encoder_raw[3];
uint32_t HAL_GetTick(void) {return fake_ms;}
uint32_t micros(void) {return fake_us;}
void delay_ms(unsigned short ms) {fake_ms+=ms;fake_us+=ms*1000;}
void delay_us(unsigned long us) {fake_us+=(uint32_t)us;fake_ms=fake_us/1000;}
uint8_t USART_ReadLine(char *line) {if(!input_line)return 0;strcpy(line,input_line);input_line=NULL;return 1;}
void USART_ClearCommands(void) {}
uint16_t USART_TxFree(void) {return 1023;}
uint16_t ENC_CountsPerRev(uint8_t idx) {return idx<3?16384:0;}
uint8_t ENC_GetAngle(uint8_t idx,int16_t *raw,float *deg) {*raw=encoder_raw[idx];*deg=*raw*360.0f/16384;return !encoder_ok;}
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
    int i; safety_trip=0;mock_gpio.IDR=GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
    LQR_Balance_Init();SimpleFOC_PID_Init();voltage_power_supply=12;voltage_limit=5;velocity_limit=60;pole_pairs=7;
    imu_online=1;Control_Heartbeat();main_alive_ms=fake_ms;
    for(i=0;i<3;i++) {motor_aligned[i]=sensor_valid[i]=1;encoder_raw[i]=0;sensor_direction[i]=1;}
    command("APP 0");command("DBG 1");
}
int main(void) {
    int i; float u;
    setup();command("DBG 2");command("EN1 1");assert(!m1_enabled&&!m2_enabled);
    command("EN2 1");assert(m2_enabled&&!m1_enabled&&!m3_enabled);
    command("M1 20");assert(target_m1==0);command("M 0 3 0");assert(target_m2==3);
    command("M2 nan");assert(target_m2==3);command("M2 8junk");assert(target_m2==3);
    command("APP 1");assert(!m1_enabled&&!m2_enabled&&!m3_enabled&&target_m2==0);
    command("EN2 1");command("M2 20");assert(!m2_enabled&&target_m2==0);
    command("BAL 1");assert(cubli_bal.state==BAL_STATE_DISABLED);
    command("CAL_IMU");
    for(i=0;i<200;i++) {float a[3]={0,0,1},w[3]={0};advance();LQR_Attitude_Update(a,w,.005f);}
    assert(cubli_bal.attitude.is_calibrated&&cubli_bal.state==BAL_STATE_DISABLED&&!m1_enabled);
    command("BMODE 4");command("LQR 28 2.2 .07 .006");assert(LQR_GetGains(0)->K_theta==28);
    command("LQR3 29 2.3 -.02 .001");assert(cubli_bal.corner_gains[2].K_w==-.02f);
    gravity(.577350269f,.577350269f,.577350269f);command("BAL 1");assert(cubli_bal.state==BAL_STATE_STANDBY);
    LQR_Motor_Loop();advance();gravity(.577350269f,.577350269f,.577350269f);LQR_Balance_Loop();
    assert(cubli_bal.state==BAL_STATE_BALANCING&&m1_enabled&&m2_enabled&&m3_enabled);
    command("BAL 0");assert(!m1_enabled&&!m2_enabled&&!m3_enabled&&target_m1==0);
    command("BMODE 1");gravity(0,sinf(.02f),cosf(.02f));command("BAL 1");advance();gravity(0,sinf(.02f),cosf(.02f));LQR_Motor_Loop();LQR_Balance_Loop();
    assert(m1_enabled&&!m2_enabled&&!m3_enabled);u=cubli_bal.control_u[0];assert(u>0);
    /* Encoder polarity must not change the mechanical feedback direction. */
    sensor_direction[0]=-1;advance();gravity(0,sinf(.02f),cosf(.02f));LQR_Balance_Loop();assert(fabsf(cubli_bal.control_u[0]-u)<.001f);
    advance();gravity(0,sinf(.4f),cosf(.4f));LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_FALL_PROTECT&&!m1_enabled);
    advance();gravity(0,0,1);LQR_Balance_Loop();assert(cubli_bal.state==BAL_STATE_FALL_PROTECT);
    command("BAL 1");advance();gravity(0,0,1);LQR_Motor_Loop();LQR_Balance_Loop();assert(m1_enabled);
    fake_ms+=31;LQR_Motor_Loop();assert(cubli_bal.state==BAL_STATE_FALL_PROTECT&&!m1_enabled);
    setup();command("EN1 1");fake_ms+=1501;main_alive_ms=fake_ms;Control_SafetyTick();assert(safety_trip==2&&!m1_enabled);
    setup();command("EN1 1");mock_gpio.IDR&=~GPIO_PIN_9;Control_SafetyTick();assert(safety_trip==3&&!m1_enabled);
    setup();command("EN1 1");fake_ms+=31;Control_Heartbeat();Control_SafetyTick();assert(safety_trip==5&&!m1_enabled);
    setup();command("MODE 1");command("EN1 1");encoder_ok=0;move(3,0);assert(safety_trip==6);encoder_ok=1;
    setup();motor_aligned[0]=1;assert(Motor_alignSensor(0)!=0&&!motor_aligned[0]&&!m1_enabled); /* stationary encoder cannot pass */
    setup();locked_rotor_since[0]=0;controller=Type_velocity_openloop;
    voltage[0].q=5;shaft_velocity[0]=0;move(3,0);fake_us+=805000;fake_ms+=805;move(3,0);assert(safety_trip==7);
    setup();velocityOpenloop(0,0);assert(TIM1->CCR1==TIM1->CCR2&&TIM1->CCR2==TIM1->CCR3);
    command("EN1 1");Control_RequestStop(1);setPhaseVoltage(5,0,1,0);assert(!m1_enabled&&TIM1->CCR1==0);
    setup();fake_ms=UINT32_MAX-100;Control_Heartbeat();command("EN1 1");fake_ms+=1501;main_alive_ms=fake_ms;Control_SafetyTick();assert(safety_trip==2);
    puts("PASS: firmware mode, calibration, geometry, fall latch, sensor, driver, heartbeat and overflow-clock regressions");
    return 0;
}
