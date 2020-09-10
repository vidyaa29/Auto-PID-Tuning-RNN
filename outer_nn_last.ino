#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <Wire.h>
/*********************************************
 * CONSTANTS DECLARATION
 *********************************************/

#define WHEEL_DIAMETER          0.1956   //m
#define BASE_WIDTH              0.46     //m
#define TICKS_PER_ROTATION_L    225
#define TICKS_PER_ROTATION_R    225
#define CONTROL_UPDATE_INTERVAL 0.05
#define MIN_PWM 530
#define MAX_PWM 1000

/********************************************
 * Declaration for encoders and flag rivving filters
 *********************************************/

volatile unsigned long corr_ticks_l, corr_prev_ticks_l = 0, ticks_l = 0, overflowcntl = 0;
volatile unsigned long corr_ticks_r, corr_prev_ticks_r = 0, ticks_r = 0, overflowcntr = 0;
unsigned long corr_l, corr_r, prev_ticks_l = 0, prev_ticks_r = 0;
unsigned long pres_time, prev_time;
double elapsed_time = 0;
double curr_rpm_l = 0, curr_rpm_r = 0, curr_vel_l = 0, curr_vel_r = 0, prev_curr_vel_l = 0, prev_curr_vel_r = 0;
double output_l = 555, output_r = 555;
const int left_rev_pin = 6, right_rev_pin = 7;
const int left_pwm_pin = 12, right_pwm_pin = 11;
float beta_vel = 0.7, beta_ang = 0.7;
int tim = 1;
int flag_rivving = 1;
double ts;
double tr ;
double eos ;

int rev_flag_l = 1, rev_flag_r = 1, prev_rev_flag_l = 1, prev_rev_flag_r = 1;  // 1 => forward, -1 => backward
/**********
 * PID
 **********/

double l,r,prev_l=0,prev_r=0, error_l=0, error_r=0, d_error_l=0, d_error_r=0, i_error_r=0, i_error_l=0, o_error=0;
/*************************************
 * declare kp, kd, ki values
 **************************************/
double kpl = 1, kil = 0.015, kdl = 0,kpl_o=1, kil_o = 0.015, kdl_o, ko_o;
double kpr = 5, kir = 0.01, kdr = 0,ko=1,kpr_o,kir_o,kdr_o;
double set = 0.9;
double k = 64.8;
double wn=2.42, zeta=2.59,m_zeta;
 double tow = 0.11;
double error=INFINITY, cost_fn = INFINITY, th[6] = {1,1,1,0.1,0.1,0.1}, sum_1 =0,sum_2 = 0, alpha = 0.5;
double c_1, c_2, c, m;

void setupPWM16() {
  DDRB |= _BV(PB1)|_BV(PB2);
  TCCR1A = _BV(COM1A1)| _BV(COM1B1) |_BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1 = 1023;
}
void init_counter() { 
  TCNT4 = 0; // Initially the count is put 0.
  TCNT5 = 0; // Initially the count is put 0.
  
  TCCR4A = 0; // initial reset
  TCCR5A = 0;
  TCCR4B = 0; // initial reset
  TCCR5B = 0;
  
  TIMSK4 = _BV (TOIE4); // over flow flag
  TIMSK5 = _BV (TOIE5);   
    
  TCCR4B =  _BV (CS40) | _BV (CS41) | _BV (CS42); 
  TCCR5B =  _BV (CS50) | _BV (CS51) | _BV (CS52); // configure this timer in external clock mode so that we can use it as a counter instead if timer.
}

ISR (TIMER4_OVF_vect) {
  ++overflowcntl; //count the number of overflows because timer can count max about 2^16 but our encoder produces even many number of ticks.
}

ISR (TIMER5_OVF_vect) {
  ++overflowcntr; //count the number of overflows because timer can count max about 2^16 but our encoder produces even many number of ticks.
}


void curr_velocity_rpm_act_ang_vel() {
  curr_vel_l = double((corr_ticks_l - corr_prev_ticks_l) * PI * WHEEL_DIAMETER) / double(TICKS_PER_ROTATION_L * elapsed_time);
  curr_vel_r = double((corr_ticks_r - corr_prev_ticks_r) * PI * WHEEL_DIAMETER) / double(TICKS_PER_ROTATION_R * elapsed_time);
  curr_vel_l = (1 - beta_vel)*curr_vel_l + beta_vel*prev_curr_vel_l;
  curr_vel_r = (1 - beta_vel)*curr_vel_r + beta_vel*prev_curr_vel_r;
  
  prev_curr_vel_l = curr_vel_l;
  prev_curr_vel_r = curr_vel_r;
}
void err()
{
  prev_l = error_l;
  prev_r = error_r;
  error_l = set - curr_vel_l;
  error_r = set - curr_vel_r;
  d_error_l = error_l - prev_l;
  d_error_r = error_r - prev_r;
  i_error_l += error_l;
  i_error_r += error_r;
  o_error = (curr_vel_l - curr_vel_r)/2;
  
}
void update_pid()
{
  // updates the kp, ki values if the model response error is lesser than the threshold
  if(cost_fn < 0.001)
  {
    //Serial.print("inside\n");
    kpl_o = kpl;
    kil_o = kil;
    kdl_o = kdl;
    ko_o = ko;
    kpr_o = kpr;
    kir_o = kir;
    kdr_o = kdr;
    
    
    
  }
}

 
 
void update_dummy_pid()
{
   // closed loop transfer function

  //  t(s)= kp *s + ki
//         ---------------------- 
//         tow/k *s2 + kp *s + ki

//representing in the form of second order eqns 
  wn = sqrt((k*kil)/tow);
zeta = 0.5* kpl * sqrt(k/(tow*kil));
m_zeta = sqrt(abs(pow(zeta,2) - 1));
  ts = 4/(zeta * wn);
  tr = 3.14 / (2* wn);
 eos = exp(-1*zeta*3.14/m_zeta);// 1 - tow/0.6; 

  kpl = tr * th[0] + ts*th[1] + eos * th[2];
  kil = tr* th[3] + ts* th[4] + eos * th[5];

  c_1 = 1 + 1/(2*( zeta + m_zeta ) * m_zeta) * exp(-1* (zeta*wn + wn*m_zeta)*CONTROL_UPDATE_INTERVAL*6*zeta);  //consider settling time as 6 times tow
  c_2 = 1/(2*( zeta - m_zeta ) * m_zeta) * exp(-1* (zeta*wn - wn*m_zeta)*CONTROL_UPDATE_INTERVAL*6*zeta);
  c = c_1 - c_2;
  
}
void weight_update() //method 1
{
  //updation using backprop 
  sum_1= th[0] + th[1] + th[2];
  for (int i=0;i<3;i++)
  th[i] = th[i] - ((alpha/m) * (error * kpl)*th[i])/sum_1;

  sum_2 = th[3] + th[4] + th[5];
  for (int j=0;j<3;j++)
  th[3+j] = th[3 + j] - ((alpha/m) * (error* kil)*th[3 + j])/sum_2;
  
}
void cost_fun()
{
  error = c - set;
  m = tow/k;
  cost_fn = 1/(2*m) * pow(error,2);
  
}
void pid()
{
  output_l = output_l + kpl_o * error_l + kil_o * i_error_l*0.05 + kdl_o* d_error_l/0.05 - ko_o * o_error;
  output_r = output_r + kpr_o * error_r + kir_o * i_error_r*0.05 + kdr_o* d_error_r/0.05 + ko_o * o_error; 
}
void analogWrite16(uint8_t pin, uint16_t val) {
  switch (pin) {
    case 12: OCR1B = val; break;
    case 11: OCR1A = val; break;
  }
}

void setup() {
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(35, OUTPUT);
  pinMode(left_rev_pin, OUTPUT);
  pinMode(right_rev_pin, OUTPUT);
  /*digitalWrite(left_pwm_pin, LOW);
  digitalWrite(right_pwm_pin, LOW);*/
  digitalWrite(left_rev_pin, HIGH);
  digitalWrite(right_rev_pin, HIGH);
  
  init_counter();
  setupPWM16();
  //Serial.begin(57600);
//  nh.initNode();
//  nh.getHardware()->setBaud(57600);
Serial.begin(57600);
  TCNT4 = 0;
  TCNT5 = 0;
  ticks_l = (overflowcntl << 16) + TCNT4;
  ticks_r = (overflowcntr << 16) + TCNT5;
  corr_r = ticks_r;
  corr_l = ticks_l;
  
  prev_ticks_l = ticks_l;
  prev_ticks_r = ticks_r;
  //handle_cmd(0.6, 0);
  ts = 4/(zeta * wn);
  tr = 3.14 / (2* wn);
 eos = exp(-1*zeta*3.14/m_zeta);// 1 - tow/0.6; 

}


void loop() {  

  //nh.spinOnce();

  if(flag_rivving == 1){
    delay(4000);
    TCNT4 = 0;
    TCNT5 = 0;
    flag_rivving = 0;
    prev_time= millis();
  }
  
  ticks_l = (overflowcntl << 16) + TCNT4;
  ticks_r = (overflowcntr << 16) + TCNT5;
  
  if(ticks_l - prev_ticks_l > 10){
    corr_l  += ticks_l - prev_ticks_l;
  }
  if(ticks_r - prev_ticks_r > 10){
    corr_r  += ticks_r - prev_ticks_r ;
  }

  pres_time = millis();
  elapsed_time = pres_time - prev_time;
  elapsed_time = elapsed_time/1000; // convert to seconds for later calculations
  if (elapsed_time >= CONTROL_UPDATE_INTERVAL) {
    prev_time = millis();
    corr_ticks_l = ticks_l - corr_l;
    corr_ticks_r = ticks_r - corr_r;
    curr_velocity_rpm_act_ang_vel();
   
    update_dummy_pid();
    cost_fun();

    weight_update();
    update_pid();    //update kp, ki values if the cost function is less than the threshold
    err();
    pid();


    //output_l=565;
    //output_r=573;
   analogWrite16(12, output_l);
   analogWrite16(11, output_r); 
    
    Serial.print(curr_vel_l);
    Serial.print("\t");
    Serial.print(kpl);
    Serial.print("\t"); 
    Serial.print(kil);
    Serial.print("\t");
    Serial.print(tr);
    Serial.print("\n");
   
    corr_prev_ticks_l = corr_ticks_l;
    corr_prev_ticks_r = corr_ticks_r;
    
    beta_vel = 0.7 + (0.20/40)*tim;
    beta_ang = 0.10 + (0.60/40)*tim;
    if(tim < 40){
      tim++;
    }

  }
  
  prev_ticks_l = ticks_l;
  prev_ticks_r = ticks_r;  
}
