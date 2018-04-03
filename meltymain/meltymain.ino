#include <math.h>
#include <Servo.h>

#define DELTA_V_TO_BAL .01 //voltage difference between highest and lowest cell for balancing to engage
#define MIN_VBATT_TO_BAL 3.8

#define RC_PWM_INPUTS 5
#define RC_PWM_TIMEOUT 25000 //ms

//pinouts
#define X_IN 0
#define Y_IN 1
#define RPM_IN 2
#define OPT1_IN 3 //shows up as optional 1 on the pcb, 
#define OPT2_IN 4 

#define LED_EN 5
#define HTR_EN 6
#define ACC_STATUS 7
#define ACC_SELF_TEST 8
#define C1_DRAIN 9
#define C2_DRAIN 10
#define C3_DRAIN 11
#define C4_DRAIN 12
#define V_BATT_SENSE_EN 13
#define C4_V 14
#define C3_V 15
#define C2_V 16
#define C1_V 17
#define ACC_X_OUT 18
#define ACC_Y_OUT 19
#define ESC_EN 20 //actually an input, not output
#define ACC_OUT 21
#define TC_OUT 22
#define ESC_OUT 23

double pi_d = 3.1415926535897932384626433832795;
float pi = pi_d;
float pi_rcp = 1/pi_d; //reciprocal of pi
float two_pi = pi_d*2;
float kRadToDeg = 180.0/pi_d;
float kDegToRad = pi_d/180.0;

boolean debug = true;

Servo esc;

//loop variables
uint8_t mode; //0 = idle, 1=balancing, 2=combat

//inputs/sensors
uint32_t iter; //loop execution counter
uint32_t t_now; //time in micros (since boot)
uint32_t t_last; //last loop iteration time 
float dt; //t_now-t_last (converted to seconds)
float vbus;
uint16_t acc_value;
uint32_t acc_value_avglen;
uint64_t acc_value_avg;
boolean orient = false; //false = rightsideup, true = upsidedown
uint16_t orient_val = 0;
const uint16_t orient_RSU_thresh = 502;
const uint16_t orient_USD_thresh = 522; 

const uint8_t rc_in_pins[] = {X_IN, Y_IN, RPM_IN, OPT1_IN, OPT2_IN};
volatile uint32_t pEdge[RC_PWM_INPUTS]; //front pulse edge
volatile boolean pHigh[RC_PWM_INPUTS];
volatile uint16_t pWidth[RC_PWM_INPUTS]; //width of pulse

uint16_t x_in; //input pwm from rc reciever, microsecond pulse widths of around 1000-2000 us
uint16_t y_in;
uint16_t rpm_in;
uint16_t rot_in;
uint16_t trim_in;
uint16_t mode_in;

//motion variables
float omega; //radians per second
float theta; //radians from north
uint16_t m_out; //motor output in pwm output

//user io gains and consts
float translate_joy_gain; //ratio of joystick input to motor modulation amplitude
float translate_phase_shift;
float rotate_joy_gain;

float led_fan_width; //the width of the led blink as the bot rotates (in radians)
float led_fan_boundH;
float led_fan_boundL;

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
float cell_v_ratio[4] = {1.4824,3.1385,4.055,7.91}; //multiply the analog reading by this to get the end voltage, set this to the resistor divider factor, will convert this to 0-1023 in setup
uint8_t cell_v_pins[4] = {C1_V, C2_V, C3_V, C4_V};
uint8_t cell_drain_pins[4] = {C1_DRAIN, C2_DRAIN, C3_DRAIN, C4_DRAIN};

void setup() {
  Serial.begin(9600);

  esc.attach(ESC_OUT);
  esc.writeMicroseconds(1000);

  led_fan_width = degToRad(30);
  led_fan_boundH = degToRad(90)+led_fan_width/2.0;
  led_fan_boundL = degToRad(90)-led_fan_width/2.0;

  acc_center = 430;
  acc_k = 70;
  acc_plus_c = 30;
  acc_trim = 0;
  

  translate_joy_gain = 100.0 / 500.0; //target peak us divided by the +- us of the joystick input, no idea of the actual units
  translate_phase_shift = pi_d/2.0;
  rotate_joy_gain = 4.0*pi_d / 500.0; //radians per second (full joy rotation) / 500 gets us the 
  
  mode = 0;
  iter = 0;
  t_now = 0;
  t_last = 0;
  vbus = 0;

  for(uint8_t i = 0; i<4; i++) { cell_v_ratio[i] = 3.3*cell_v_ratio[i]/1024.0; voltages[i] = readBattVoltage(i); }

  pinMode(X_IN, INPUT);
  pinMode(Y_IN, INPUT);
  pinMode(RPM_IN, INPUT);
  pinMode(OPT1_IN, INPUT);
  pinMode(OPT2_IN, INPUT);
  pinMode(ACC_STATUS, INPUT);
  pinMode(C1_V, INPUT);
  pinMode(C2_V, INPUT);
  pinMode(C3_V, INPUT);
  pinMode(C4_V, INPUT);
  pinMode(ACC_X_OUT, INPUT);
  pinMode(ACC_Y_OUT, INPUT);
  pinMode(ESC_EN, INPUT);
  pinMode(ACC_OUT, INPUT);
  pinMode(TC_OUT, INPUT);

  pinMode(LED_EN, OUTPUT);
  pinMode(HTR_EN, OUTPUT);
  pinMode(ACC_SELF_TEST, OUTPUT);
  pinMode(C1_DRAIN, OUTPUT);
  pinMode(C2_DRAIN, OUTPUT);
  pinMode(C3_DRAIN, OUTPUT);
  pinMode(C4_DRAIN, OUTPUT);
  pinMode(V_BATT_SENSE_EN, OUTPUT);
  pinMode(ESC_OUT, OUTPUT);

  attachInterrupt(rc_in_pins[0], isrPTimer0, CHANGE);
  attachInterrupt(rc_in_pins[1], isrPTimer1, CHANGE);
  attachInterrupt(rc_in_pins[2], isrPTimer2, CHANGE);
  attachInterrupt(rc_in_pins[3], isrPTimer3, CHANGE);
  attachInterrupt(rc_in_pins[4], isrPTimer4, CHANGE);

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
      esc.writeMicroseconds(1000);
      digitalWrite(LED_EN, LOW);
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
      delay(1000);
      break;

    case(1): //balancing
      esc.writeMicroseconds(1000);
      digitalWrite(V_BATT_SENSE_EN, HIGH);
      digitalWrite(LED_EN, LOW);
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
          digitalWrite(LED_EN, HIGH); //turn on leds, indicating that battery is currently balancing
        }
      }
      if(debug) Serial.println();
      
      delay(990); //drain the cells for 250 ms
      for(int i = 0; i<4; i++) digitalWrite(cell_drain_pins[i], LOW);//stop pulling current so that we can get a good voltage reading off the cells
      delay(5);
      break;

      
    case(2): //combating
      //digitalWrite(V_BATT_SENSE_EN, HIGH);
      
      x_in = pWidth[0];
      y_in = pWidth[1];
      rpm_in = pWidth[2];
      rot_in = pWidth[3];
      trim_in = pWidth[3];

      //check orientation
      if(iter%10==0) orient_val = analogRead(ACC_X_OUT);
      if(orient_val < orient_RSU_thresh) orient = false; //bot is right side up
      if(orient_val > orient_USD_thresh) orient = true; //bot is up side down
      digitalWrite(V_BATT_SENSE_EN, orient);
      if(iter%1000==0) Serial.println(orient_val);

      //v_bus = cell_v_ratio[3]*analogRead(C4_V);
      acc_value = analogRead(ACC_OUT);
      
      if(x_in < 1250) { acc_value_avg += acc_value; acc_value_avglen++; }
      else if (x_in > 1750) { acc_value_avg = 0; acc_value_avglen = 0; }
      //if(debug && iter%100 == 0) { Serial.println(); Serial.println(acc_value); }

      //acc_max = 389; dAcc = 41; rpm = 680; rad/s = 71.2094334;
      //acc_max = 346; dAcc = 84; rpm = 1010; rad/s = 105.76695255;
      //acc_max = 273; dAcc = 157; rpm = 1246; rad/s = 130.48081473;
      //acc_max = 173; dAcc = 257; rpm = 1530; rad/s = 160.22122515;
      //acc_max = 71; dAcc = 359; rpm = 1760; rad/s = 184.3067688;
      //acc_max = 36; dAcc = 394; rpm = 1805; rad/s = 189.019157775;
      //acc_max = 1; rpm = 1940;
      
      dt = (t_now-t_last)*0.000001;

      if(acc_value < 425){ //only do spinning maths and such above a certain rpm
        m_out = rpm_in;// - translate_joy_gain;
        acc_trim = (trim_in-1500.0)*(-.002);
        
        //integrate the accelerometer value to heading
        omega = sqrt( abs(acc_value-acc_center) * acc_k) + acc_plus_c + acc_trim; 
        //omega += (rot_in-1500)*rotate_joy_gain;
        theta += omega*dt;
        
        //x, modify m_out by sin_theta times the x joystick gain, times the translate gain
        //uncomment once done with averaging things
        //m_out += (int16_t)(sin(theta+translate_phase_shift) * (x_in-1500) * translate_joy_gain);
        //y
        m_out += (int16_t)(cos(theta+translate_phase_shift) * (y_in-1500) * translate_joy_gain);

        if(m_out > 2000) m_out = 2000;
        else if(m_out < 1000) m_out = 1000;
        esc.writeMicroseconds(m_out);

        //we don't want theta to accumulate too much and overflow (or run out of precision? for floating point?)
        if(theta > two_pi) theta -= two_pi;
        if(theta < 0) theta += two_pi; //keep value positive
        
        //check if LEDs need to be turned on
        digitalWrite(LED_EN, (theta > led_fan_boundL && theta < led_fan_boundH)?(HIGH):(LOW));  
      } else {
        digitalWrite(LED_EN, LOW);
        esc.writeMicroseconds(rpm_in);
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

