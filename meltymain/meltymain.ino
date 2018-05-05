#include <math.h>
#include "kinetis.h"

#define DELTA_V_TO_BAL .01 //voltage difference between highest and lowest cell for balancing to engage
#define MIN_VBATT_TO_BAL 3.8

#define RC_PWM_INPUTS 8
#define RC_PWM_TIMEOUT 25000 //ms

#define ONESHOT_MAX_HZ 3000

//pinouts
#define X_IN 0
#define Y_IN 1
#define RPM_IN 2
#define ESC_OUT 3
#define ROT_IN 4
#define MODE_IN 5 //was formerly OPT1, which is why we start at OPT2
#define OPT2_IN 6 //will be used for trim
#define OPT3_IN 7
#define OPT4_IN 8
//9 no connection
//10 nc
#define DBG_LED2 11
#define DBG_LED1 12
#define V_BATT_SENSE_EN 13
#define C4_V 14
#define C3_V 15
#define C2_V 16
#define C1_V 17
#define ACC_X_OUT 18
#define ACC_Y_OUT 19
#define ACC_Z_OUT 20
#define ESC_EN 21 //actually an analog input, not output
#define ACC_OUT 22
#define TC_OUT 23
//24 nc
#define C1_DRAIN 25
#define C2_DRAIN 26
#define C3_DRAIN 27
#define C4_DRAIN 28
#define GLED_EN 29
#define RLED_EN 30
#define ACC_SELF_TEST 31
#define ACC_STATUS 32
#define HTR_EN 33

double pi_d = 3.1415926535897932384626433832795;
float pi = pi_d;
float pi_rcp = 1/pi_d; //reciprocal of pi
float two_pi = pi_d*2;
float kRadToDeg = 180.0/pi_d;
float kDegToRad = pi_d/180.0;

boolean debug = true;

//loop variables
uint8_t mode; //0 = idle, 1=balancing, 2=combat

//inputs/outputs/sensors
uint32_t iter; //loop execution counter
uint32_t t_now; //time in micros (since boot)
uint32_t t_last; //last loop iteration time 
float dt; //t_now-t_last (converted to seconds)
float vbus;
const float vbus_aread_const = 7.91; //multiply this by the value of the analogRead(ESC_EN) to get the voltage to the esc
uint16_t acc_value;
uint32_t acc_value_avglen;
uint64_t acc_value_avg;
boolean orient = false; //false = rightsideup, true = upsidedown
uint16_t orient_val = 0;
const uint16_t orient_RSU_thresh = 502;
const uint16_t orient_USD_thresh = 522; 

const uint8_t rc_in_pins[] = {X_IN, Y_IN, RPM_IN, ROT_IN, MODE_IN, OPT2_IN, OPT3_IN, OPT4_IN};
volatile uint32_t pEdge[RC_PWM_INPUTS]; //front pulse edge
volatile boolean pHigh[RC_PWM_INPUTS];
volatile uint16_t pWidth[RC_PWM_INPUTS]; //width of pulse

const uint32_t oneshot_write_dt = (uint32_t)( 1000000.0f / ONESHOT_MAX_HZ ); //micros
uint32_t oneshot_last_write; 

uint16_t x_in; //input pwm from rc reciever, microsecond pulse widths of around 1000-2000 us
uint16_t y_in;
uint16_t rpm_in;
uint16_t mode_in;
uint16_t rot_in;
uint16_t trim0_in;
uint16_t trim1_in;
uint16_t extra_in;

//motion variables
float omega; //radians per second
float theta; //radians from north
float m_out; //motor output from 0 to 1

//user input gains and consts
float translate_joy_gain; //ratio of joystick input to motor modulation amplitude
float translate_phase_shift;
float rotate_joy_gain;

float led_fan_width; //the width of the led blink as the bot rotates (in radians)
float GLED_fan_boundH;
float GLED_fan_boundL;
float RLED_fan_boundH;
float RLED_fan_boundL;

uint16_t acc_center;
float acc_k;
float acc_plus_c;
float acc_trim;

//balancing variables
float voltagesK_LPF = .1;
float voltages[4]; //relative to ground
float cellVoltages[4]; //relative to eachother
uint8_t c_low; //lowest voltage cell number (0 to 3)

//resistor divider ratios
//{1.4745,3.1357,4.0321,8.0819}; legit
//{1.47999,3.13,4.0321,7.885};
//3.965, 7.94, 11.93, 15.99
float vbus_v_ratio = 1;
float cell_v_ratio[4] = {1.4824,3.1385,4.055,7.91}; //multiply the analog reading by this to get the end voltage, set this to the resistor divider factor, will convert this to 0-1023 in setup
uint8_t cell_v_pins[4] = {C1_V, C2_V, C3_V, C4_V};
uint8_t cell_drain_pins[4] = {C1_DRAIN, C2_DRAIN, C3_DRAIN, C4_DRAIN};

void setup() {
  Serial.begin(9600);

  led_fan_width = degToRad(30);
  GLED_fan_boundH = degToRad(90)+led_fan_width/2.0;
  GLED_fan_boundL = degToRad(90)-led_fan_width/2.0;
  RLED_fan_boundH = GLED_fan_boundH + degToRad(90);
  RLED_fan_boundL = GLED_fan_boundL + degToRad(90);

  acc_center = 795.2;
  acc_k = 85;
  acc_plus_c = 0;
  acc_trim = 0;

  translate_joy_gain = 100.0 / 500.0; //target peak us divided by the +- us of the joystick input, no idea of the actual units
  translate_phase_shift = degToRad(30);
  rotate_joy_gain = 4.0*pi_d / 500.0; //radians per second (full joy rotation) / 500 gets us the 
  
  mode = 0;
  iter = 0;
  t_now = 0;
  t_last = 0;
  vbus = 0;

  pinMode(X_IN, INPUT);
  pinMode(Y_IN, INPUT);
  pinMode(RPM_IN, INPUT);
  pinMode(ESC_OUT, OUTPUT);
  pinMode(ROT_IN, INPUT);
  pinMode(MODE_IN, INPUT);
  pinMode(OPT2_IN, INPUT);
  pinMode(OPT3_IN, INPUT);
  pinMode(OPT4_IN, INPUT);
  //9 nc
  //10 nc
  pinMode(DBG_LED2, OUTPUT);
  pinMode(DBG_LED1, OUTPUT); 
  pinMode(V_BATT_SENSE_EN, OUTPUT);
  pinMode(C4_V, INPUT);
  pinMode(C3_V, INPUT);
  pinMode(C2_V, INPUT);
  pinMode(C1_V, INPUT);
  pinMode(ACC_X_OUT, INPUT);
  pinMode(ACC_Y_OUT, INPUT);
  pinMode(ACC_Z_OUT, INPUT);
  pinMode(ESC_EN, INPUT);
  pinMode(ACC_OUT, INPUT);
  pinMode(TC_OUT, INPUT);
  //25 nc
  pinMode(C1_DRAIN, OUTPUT);
  pinMode(C2_DRAIN, OUTPUT);
  pinMode(C3_DRAIN, OUTPUT);
  pinMode(C4_DRAIN, OUTPUT);
  pinMode(GLED_EN, OUTPUT);
  pinMode(RLED_EN, OUTPUT);
  pinMode(ACC_SELF_TEST, OUTPUT);
  pinMode(ACC_STATUS, INPUT);
  pinMode(HTR_EN, OUTPUT);

  for(uint8_t i = 0; i<4; i++) { cell_v_ratio[i] = 3.3*cell_v_ratio[i]/1024.0; voltages[i] = readBattVoltage(i); }

  attachInterrupt(rc_in_pins[0], isrPTimer0, CHANGE);
  attachInterrupt(rc_in_pins[1], isrPTimer1, CHANGE);
  attachInterrupt(rc_in_pins[2], isrPTimer2, CHANGE);
  attachInterrupt(rc_in_pins[3], isrPTimer3, CHANGE);
  attachInterrupt(rc_in_pins[4], isrPTimer4, CHANGE);
  attachInterrupt(rc_in_pins[5], isrPTimer5, CHANGE);
  attachInterrupt(rc_in_pins[6], isrPTimer6, CHANGE);
  attachInterrupt(rc_in_pins[7], isrPTimer7, CHANGE);

  configOneShot125();
  
  digitalWrite(V_BATT_SENSE_EN, HIGH);
  for(int n = 0; n<1000; n++) {
    for(int i = 0; i<4; i++){
      voltages[i] = readBattVoltage(i)*voltagesK_LPF + voltages[i]*(1-voltagesK_LPF);
    }
    delay(1);
  }
  digitalWrite(V_BATT_SENSE_EN, LOW);
}

void isrPTimer0() { isrPTimer(0); }
void isrPTimer1() { isrPTimer(1); }
void isrPTimer2() { isrPTimer(2); }
void isrPTimer3() { isrPTimer(3); }
void isrPTimer4() { isrPTimer(4); }
void isrPTimer5() { isrPTimer(5); }
void isrPTimer6() { isrPTimer(6); }
void isrPTimer7() { isrPTimer(7); }

void isrPTimer(uint8_t ch) {
  if(digitalRead(rc_in_pins[ch]) == HIGH ) {
    pEdge[ch] = micros();
    pHigh[ch] = true;
  } else if (pHigh[ch] == true) { //if pin was previously high
    pWidth[ch] = micros() - pEdge[ch];
    if(pWidth[ch] > RC_PWM_TIMEOUT) pWidth[ch] = 0;
    pHigh[ch] = false;
  }
}

void loop() {

  t_last = t_now;
  t_now = micros();

  //the following function checks if the pwm inputs have been updated recently enough
  //this is done very strangely, because sometimes the interrupt can happen such that t_now is smaller than pEdge
  //done normally (t_now-pEdge > RC_PWM_TIMEOUT) would result in occasions where t_now-pEdge is negative, and bc unsigned it wraps around to
  //close to 2^32 which is of course going to be much bigger than RC_PWM_TIMEOUT, the weird fix allows t_now-pEdge to become negative, 
  //and then takes the absolute value of it, allowing pWidth to be what it likes so long as t_now and pEdge are roughly the same.
  //this comes with the caveat that if the rc reciever lost signal, and the clock wrapped around such that t_now was close to pEdge again, 
  //pWidth would not be forced to stay at 0, though this shouldn't be a problem as pWidth should have been previously set to 0,
  //and should stay that way so long as the interrupt doesn't occour again, in which case we would allow the interrupt to set pWidth
  for(uint8_t ch = 0; ch < RC_PWM_INPUTS; ch++) {
    if(abs( (int32_t)t_now - (int32_t)pEdge[ch] ) > 2*RC_PWM_TIMEOUT) pWidth[ch] = 0;
  }
  
  mode_in = pWidth[4];
  if(mode_in == 0) { 
    if(debug) Serial.println("No Comms");
    if(mode == 2) mode = 0;
  }
  else if(mode_in < 1250) mode = 0;
  else if(mode_in > 1750) mode = 2;
  else mode = 1;
  
  switch(mode){
    case(0): //idling
      writeOneShot125(0);
      
      digitalWrite(GLED_EN, HIGH); 
      if(mode_in == 0) digitalWrite(RLED_EN, (iter % 2 == 0)?(LOW):(HIGH));
      else digitalWrite(RLED_EN, LOW);
      
      digitalWrite(V_BATT_SENSE_EN, LOW);
      digitalWrite(HTR_EN, LOW);
      digitalWrite(ACC_SELF_TEST, LOW);
      digitalWrite(C1_DRAIN, LOW);
      digitalWrite(C2_DRAIN, LOW);
      digitalWrite(C3_DRAIN, LOW);
      digitalWrite(C4_DRAIN, LOW);
      digitalWrite(V_BATT_SENSE_EN, LOW);
      if(debug) { Serial.println(); Serial.print("Disabled; iter: "); Serial.println(iter); }
      if(acc_value_avglen > 0) Serial.println( (double)acc_value_avg/(double)acc_value_avglen );
      else Serial.println("0");
      delay(1000);
      break;

    case(1): //balancing
      writeOneShot125(0);
      digitalWrite(V_BATT_SENSE_EN, HIGH);
      digitalWrite(GLED_EN, LOW);
      digitalWrite(RLED_EN, LOW);
      delay(5);
      //there should be no drains on right now, reading the voltage is fine

      for(int i = 0; i<4; i++){
        voltages[i] = readBattVoltage(i)*voltagesK_LPF + voltages[i]*(1-voltagesK_LPF);
        //Serial.println((uint32_t)(voltages[i]*10000.0));
      }
      digitalWrite(V_BATT_SENSE_EN, LOW);
      
      cellVoltages[0] = voltages[0];
      c_low = 0;
      for(int i = 1; i<4; i++) { //read voltages and find the lowest voltage cell 
        cellVoltages[i] = voltages[i]-voltages[i-1];
        if(cellVoltages[i] < cellVoltages[c_low]) c_low = i;
      }
      
      //drain cells that are DELTA_V_TO_BAL volts above the lowest cell voltage
      for(int i=0; i<4; i++) {
        if(debug) Serial.println((uint32_t)(cellVoltages[i]*10000.0));  

        if(cellVoltages[i] > cellVoltages[c_low]+DELTA_V_TO_BAL) {
          digitalWrite(cell_drain_pins[i], HIGH);
          digitalWrite(RLED_EN, HIGH); //turn on leds, indicating that battery is currently balancing
        }
      }
      if(debug) Serial.println();
      
      delay(990); //drain the cells for 250 ms
      //for(int i = 0; i<4; i++) digitalWrite(cell_drain_pins[i], LOW);//stop pulling current so that we can get a good voltage reading off the cells
      delay(5);
      break;

      
    case(2): //combating
      //digitalWrite(V_BATT_SENSE_EN, HIGH);

      x_in = pWidth[0];
      y_in = pWidth[1];
      rpm_in = pWidth[2];
      rot_in = pWidth[3];
      trim0_in = pWidth[5];
      trim1_in = pWidth[6];
      extra_in = pWidth[7];
      //if(iter%1000 == 0) Serial.println(rpm_in);

      //check orientation
      if(iter%10==0) orient_val = analogRead(ACC_X_OUT);
      //if(orient_val < orient_RSU_thresh) orient = false; //bot is right side up
      //if(orient_val > orient_USD_thresh) orient = true; //bot is up side down
      //digitalWrite(DBG_LED1, orient);
      //if(iter%1000==0) Serial.println(orient_val);
      if(iter%10000==0) { //vbus = analogRead(ESC_EN)*vbus_v_ratio; 
      Serial.println(analogRead(ESC_EN)); }

      acc_value = analogRead(ACC_OUT);
      if(extra_in < 1250) { 
        acc_value_avg += acc_value; 
        acc_value_avglen++; 
        digitalWrite(DBG_LED2, ((iter%20000 < 10000)?(HIGH):(LOW))); 
      }
      else if (extra_in > 1750) { acc_value_avg = 0; acc_value_avglen = 0; digitalWrite(DBG_LED2, HIGH); }
      else { digitalWrite(DBG_LED2, LOW); }
      //if(debug && iter%1000 == 0) { Serial.println(); Serial.println(acc_value); }

      //acc_max = 389; dAcc = 41; rpm = 680; rad/s = 71.2094334;
      //acc_max = 346; dAcc = 84; rpm = 1010; rad/s = 105.76695255;
      //acc_max = 273; dAcc = 157; rpm = 1246; rad/s = 130.48081473;
      //acc_max = 173; dAcc = 257; rpm = 1530; rad/s = 160.22122515;
      //acc_max = 71; dAcc = 359; rpm = 1760; rad/s = 184.3067688;
      //acc_max = 36; dAcc = 394; rpm = 1805; rad/s = 189.019157775;
      //acc_max = 1; rpm = 1940;
      
      dt = (t_now-t_last)*0.000001;

      if(acc_value < 425){ //only do spinning maths and such above a certain rpm
        m_out = (rpm_in-1000)*.00035f;//.001f;// - translate_joy_gain;
        acc_trim = (trim0_in-1500)*(-.002) + (trim1_in-1500)*(-.01);
        
        //integrate the accelerometer value to heading
        omega = sqrt( abs(acc_value-acc_center) * acc_k) + acc_plus_c + acc_trim; 
        omega -= (rot_in-1500)*rotate_joy_gain;
        theta -= omega*dt; //because I actually milled the shell such that we need to spin clockwise, not counterclockwise
        
        //x, modify m_out by sin_theta times the x joystick gain, times the translate gain
        m_out += (cos(theta+translate_phase_shift) * ((x_in-1500)*.002f) * translate_joy_gain);
        //y
        m_out += (sin(theta+translate_phase_shift) * ((y_in-1500)*.002f) * translate_joy_gain);

        if(m_out > 1) m_out = 1.0f;
        else if(m_out < 0) m_out = 0.0f;
        writeOneShot125(m_out);

        //we don't want theta to accumulate too much and overflow (or run out of precision? for floating point?)
        if(theta > two_pi) theta -= two_pi; 
        if(theta < 0) theta += two_pi; //keep value positive
        
        //check if LEDs need to be turned on
        digitalWrite(GLED_EN, (theta > GLED_fan_boundL && theta < GLED_fan_boundH)?(LOW):(HIGH));
        digitalWrite(RLED_EN, (theta > RLED_fan_boundL && theta < GLED_fan_boundH)?(LOW):(HIGH));
        
      } else {
        digitalWrite(GLED_EN, LOW);
        digitalWrite(RLED_EN, (iter%10000 < 5000)?(LOW):(HIGH));
        
        writeOneShot125((rpm_in-1000)*.00035f);
        m_out = 0;
      }

      //maintain consistant loop hz
      //if(debug && iter%100 == 0) Serial.println(t_now-t_last);
      break;
  }
  
  iter++;
}

double radToDeg(double rads) { return kRadToDeg*rads; }
double degToRad(double deg) { return kDegToRad*deg; }

//cell range 0-3
double readBattVoltage(uint8_t cellNum) { return cell_v_ratio[cellNum]*analogRead(cell_v_pins[cellNum]); }

void configOneShot125() {
  //pinMode(ESC_OUT, OUTPUT);
  digitalWrite(ESC_OUT, LOW);

  //disable write protection
  FTM2_MODE |= FTM_MODE_WPDIS;
  //enable the FTM
  FTM2_MODE |= FTM_MODE_FTMEN;
  //init
  FTM2_MODE |= FTM_MODE_INIT;
  // set 0 before changing mod
  FTM2_SC = 0x00; 
  //Shouldn't be needed (should default to this), but just in case
  FTM2_CNTIN = 0x0000; 
  //make sure count is 0
  FTM2_CNT = 0x0000; 
  //set mod to be maxiumum value
  FTM2_MOD = 0xFFFF;
  
  /*
   * clock source 
   * 00 no clock selected, counter disabled
   * 01 system clock
   * 10 fixed frequency clock (I think 32 khz)
   * 11 external clock
   * 
   * we're going to disable the clock for now, but we will be setting it to the system clock later once we want to send out a pulse
   */  
  FTM2_SC |= FTM_SC_CLKS(0b00);

  /*  
   * set prescaler value 
   * 000---divide by 1
   * 001---divide by 2
   * 010---divide by 4
   * 011---divide by 8
   * 100---divide by 16
   * 101---divide by 32
   * 110---divide by 64
   * 111---divide by 128
   */
  FTM2_SC |= FTM_SC_PS(0b000);

  //setup channel 0, see page 783 for info on the FTMx_CnSC (Channel n Status and Control) register
  //ELSB:ELSA should be 1:1 for Output Compare, set output on match (not actually that important to us tho)
  FTM2_C0SC &= FTM_CSC_ELSB; 
  FTM2_C0SC &= FTM_CSC_ELSA;
  //don't need DMA
  FTM2_C0SC &= ~FTM_CSC_DMA;
  //MSB:MSA should be 0:1 for output compare mode (MS ~ mode select)
  FTM2_C0SC &= ~FTM_CSC_MSB; 
  FTM2_C0SC |= FTM_CSC_MSA;
  //enable interrupt for this channel
  FTM2_C0SC |= FTM_CSC_CHIE;
  
  //enable IRQ Interrupt, will call the ftm2_isr() function
  NVIC_ENABLE_IRQ(IRQ_FTM2);

  //Set the initial compare value
  FTM2_C0V = 0xFFFF;
}

void writeOneShot125(float val) {
  //don't allow more writes per second than the esc can take
  uint32_t t = micros();
  if(t-oneshot_last_write < oneshot_write_dt) return;
  oneshot_last_write = t;
  //Serial.println(val);
  
  //calculate the compareator value, put into
  //compare value = microseconds*clock megahertz 
  //I honestly have no idea why I need to divide by two here
  FTM2_C0V = (uint16_t)((((val*125)+125)*72)/2);
  FTM2_CNT = 0x0000;
  
  //pull the pin high
  digitalWriteFast(ESC_OUT, HIGH);
  
  //enable the clock by selecting the system clock as the clock source
  FTM2_SC |= FTM_SC_CLKS(0b01);
}

//the interrupt service routine for the FTM2
void ftm2_isr(void)
{
  //TODO: check if channel 0 is the channel from FTM 2 throwing the flag, not just if any channel is calling an interrupt
  // probably not ever going to be an issue though, as I don't think anything else should be using FTM2
  
  //end the pulse
  digitalWriteFast(ESC_OUT, LOW);
  //disable the counting by selecting no clock
  FTM2_SC |= FTM_SC_CLKS(0b00);
  //clear all the flags 
  FTM2_STATUS &= 0x00;
  //only channel 2's status flag can be reset with a
  //FTM2_STATUS &= ~(0b1 << channelNum)
}


