
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <Wire.h>

#define CONTROL_UPDATE_INTERVAL 0.05
#define WHEEL_DIAMETER 0.1956
#define TICKS_PER_ROTATION_L    225
#define TICKS_PER_ROTATION_R    225
double k,tow, a1, b0;
double sum_a =0,sum_b=0,a[6] = {0.25,0.25,0.25,0.25,0.25,0.25},b[6] = {0.25,0.25,0.25,0.25,0.25,0.25};
volatile unsigned long corr_ticks_l, corr_prev_ticks_l = 0, ticks_l = 0, overflowcntl = 0;
volatile unsigned long corr_ticks_r, corr_prev_ticks_r = 0, ticks_r = 0, overflowcntr = 0;
unsigned long corr_l, corr_r, prev_ticks_l = 0, prev_ticks_r = 0;
unsigned long pres_time, prev_time;
double elapsed_time = 0;
double curr_rpm_l = 0, curr_rpm_r = 0, curr_vel_l = 0, curr_vel_r = 0, prev_curr_vel_l = 0, prev_curr_vel_r = 0;
double output_l = 0, output_r = 0;
const int left_rev_pin = 6, right_rev_pin = 7;
const int left_pwm_pin = 12, right_pwm_pin = 11;
float beta_vel = 0.7, beta_ang = 0.7;
int tim = 1;
int flag_rivving = 1;
int rev_flag_l = 1, rev_flag_r = 1, prev_rev_flag_l = 1, prev_rev_flag_r = 1;  // 1 => forward, -1 => backward
int r=600,p=0,m=6,n=6;
double sum_we = 0, sum_ve=0;
double y_k,y[6],x[6],w[6] ={0.5,0.5,0.5,0.5,0.5,0.5}, v[6]={0.5,0.5,0.5,0.5,0.5,0.5},net_k, error = 0, cost_fun = INFINITY;
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
  
  //GTCCR = _BV (PSRASY);   // reset whatever be the default prescalars.   
    
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
   for(int i=0;i<n;i++)
   {x[i] = r/100;sum_b +=b[i]*x[i];}
  
  
  
  prev_ticks_l = ticks_l;
  prev_ticks_r = ticks_r;
}

void arma()
{
  
 if(p<m)
 {
  y[p] = curr_vel_l;
  sum_a += y[p];
    p++;

 }
 else
 {
  sum_a = 0;
  for(int j=0;j<m-1;j++)
  {y[j] = y[j+1]; sum_a += a[j] * y[j+1];} 
  
 y[m-1] = curr_vel_l;
 sum_a+=a[m-1]*y[m-1];
 }
 y_k = sum_a + sum_b;
 Serial.print(sum_a);
 Serial.print("\t");
 Serial.print(sum_b);
 
}

void activ()          //activation function
{
  //net_k_x = w[0] * x[0] + w[1] * x[1];
  //net_k_y = w[1]* y[0] + w[1]* y[1];
  net_k = -1 * (y_k);
  y_k = (1 - exp(net_k))/(1 + exp(net_k));
  
   
}
void cost_fn()
{
error = ((curr_vel_l) - y_k);
cost_fun = 0.5 * pow(error,2);

}
double alpha  = 0.1;
void weight_update(int i)   //using back prop and step gradient descent
{
  sum_we=0;
  for(int i=0;i<n;i++)
  {w[i] = w[i] - alpha* error * x[n - i];//* w[i]/sum_we;//* (2 * exp(net_k))/pow((1 + exp(net_k)),2);
  sum_we += w[i];
  }
}
void v_update(int j)
{
  sum_ve=0;
  for(int j=0;j<m;j++)
  {v[j] = v[j] - alpha * error * y[m - j];//*v[j] /sum_ve * (2 * exp(net_k))/pow((1 + exp(net_k)),2);
  sum_ve += v[j];
  }
}
void transform()
{
  for(int j=0;j<m;j++)
  a[j] = v[j]/2;
  
  for(int i=0;i<n;i++)
  b[i] = w[i]/2;
}
void transfer_fn()
{
//  tf_z = b[0] + b[1]* Z^ -1/(1 - a * Z ^-1);
  // z to s transformation :
  // k / tow * s + 1
// b0 + b1 z-1  ===  b0 + b1 + (b0 - b1) (sT/2)
// 1 - a1 z-1 ==== 1 - a1 + (1 + a1) (sT/2)
// we can get the relation if we substitute z equation in s equation and compare the coefficients 
b0 = b[5];
a1 = a[5];
k = 2* b0 / (1 - a1);
tow  = (1 + a1) * (CONTROL_UPDATE_INTERVAL)/(2*(1 - a1));
}
void loop() {
if( flag_rivving == 1 ){
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
   r=600;
   analogWrite16(12, r);
   analogWrite16(11, 0);
   if(cost_fun>0.01)
   {
   arma();         // making the auto regressive moving average model
   activ();        // finding the outer most layer
   cost_fn();      //minimising the error using cost fn LSE minimization 
//   weighted_errors();
   for(int k = 0;k<n;k++)
   weight_update(k); // updating the weights using steepest descent method
   for(int l = 0;l<m;l++)
   v_update(l); // updating the weights using steepest descent method
   transform();  //transform neura network weights to ARMA parameters
   transfer_fn();  // get the transfer function using the coefficients
  Serial.print('\n');
  for(int i =0 ; i<m;i++)
  {Serial.print(y[i]);
  Serial.print('\t');
  }
  Serial.print('\n');
  for(int i=0;i<n;i++)
  {
  Serial.print(b[i]);
  Serial.print('\t');
  }
  Serial.print('\n');
  
  for(int j=0;j<m;j++)
  {
    Serial.print(a[j]);
    Serial.print('\t');
  }
  Serial.print('\n');
  Serial.print(y_k);
  Serial.print('\t');
  Serial.print(net_k);
  Serial.print('\t');
  Serial.println(cost_fun);
   Serial.println(k);
   Serial.println(tow);
   }
   else
   exit(0);
   corr_prev_ticks_l = corr_ticks_l;
    corr_prev_ticks_r = corr_ticks_r;
    
    beta_vel = 0.7 + (0.20/40)*tim;
    beta_ang = 0.10 + (0.60/40)*tim;
    if(tim < 40)
      tim++;
    

  }
  
  prev_ticks_l = ticks_l;
  prev_ticks_r = ticks_r;  
  
}
