/******************************************************************
 * Environment sensing meter project
 * April 2020, shabaz, rev 1.0
 * Arduino setup:
 *   Arduino Micro was used
 *   Go to Sketch->Include Library->Manage Libraries
 *   and type Sensirion in the search box, and select
 *   "arduino-ess"
 * Pins:
 * 100uA meter connected via 47k resistor to PB5
 * LEDs connected via 220 ohm resistor to PF0, PF1, PF4, PF5
 *   (Temp, RF, VOC, CO2 respectively)
 * Push-button (mode select) connected to PF7 and ground
 * I2C (SDA[PD1] and SCL[PD0], and +5V pin and GND connected to
 *   Sensirion Environment Sensor Shield
 ******************************************************************/

#include <math.h>
#include <EEPROM.h>
#include <sensirion_ess.h>

// *************** definitions **********************
#define meter OCR1A // PB5
#define BUTTON 18
#define LED_TEMP 23
#define LED_RH 22
#define LED_VOC 21
#define LED_CO2 20
#define LED_BOARD LED_BUILTIN
#define LED_BOARD_OFF digitalWrite(LED_BOARD, LOW)
#define LED_BOARD_ON digitalWrite(LED_BOARD, HIGH)
#define MODE_TEMP 0
#define MODE_RH 1
#define MODE_VOC 2
#define MODE_CO2 3
#define MODE_NONE 99
#define VIEW_STATIC 0
#define VIEW_DYNAMIC 1
#define DEB_NONE 0
#define DEB_TIME 1
#define DEB_TIME2 2
#define IS_PRESSED LOW
#define IS_UNPRESSED HIGH
#define DEBOUNCE_TIME 4
#define METER_TIME 2
#define VIEW_EXP 1000
#define VSIZE 8






// *************** globals and constants *************
const int led_pin[]={LED_TEMP, LED_RH, LED_VOC, LED_CO2};
SensirionESS ess;
int mode=MODE_TEMP;
int view=VIEW_DYNAMIC;
int fast_tick=0;
int slow_tick=0;
int led_status=0;
int button_status=DEB_NONE;
int button_time=0;
int shimmer=0;
int view_time=0;
int update_view=0;
int meter_update_timer=0;
int changed_mode=0;
int mval[4];
int vocbuffer[VSIZE]; // VOC values are low-pass filtered
int mmin=100;
int mmax=3500;

// *************** functions *************************
// Toggle board LED as required (used for troubleshooting)
void toggle_board_led(void)
{
  if (led_status){
  LED_BOARD_OFF;
  led_status=0;
  } else {
    LED_BOARD_ON;
    led_status=1;
  }
}

// Set mode LED on
void set_mode_led(int mode) {
  int i;
  for (i=0; i<4; i++)
  {
    digitalWrite(led_pin[i], LOW);
  }
  if (mode<4) {
    digitalWrite(led_pin[mode], HIGH);
  }
}

// *************** timer tick ************************
SIGNAL(TIMER3_COMPA_vect) // this occurs 1000 times per second
{
  fast_tick++;
  if (fast_tick>100) {
    fast_tick=0;
  }
  else {
    return; // we only want a 10 msec tick
  }

  // this code executes every 10 msec
  
  // blink the mode light a few times if mode has just been set to dynamic
  if (shimmer>0) {
    shimmer++;
    if (shimmer%8) {
      set_mode_led(99);
    } else if ((shimmer+4)%8) {
      set_mode_led(mode);
      if (shimmer>32) shimmer=0;
    }
  }

  // time to update the view if we're in dynamic mode?
  if (view==VIEW_DYNAMIC) {
    view_time++;
    if (view_time>VIEW_EXP) {
      update_view=1;
    }
  }

  if (button_status==DEB_NONE) {
    if (digitalRead(BUTTON)==IS_PRESSED) {
      button_status=DEB_TIME;
      button_time=0;
      if (view==VIEW_DYNAMIC) {
        view=VIEW_STATIC;
        mode=MODE_TEMP;
      } else {
        mode++;
        if (mode>3) {
          view=VIEW_DYNAMIC;
          view_time=0;
          mode=MODE_TEMP;
          shimmer=1;
        }
      }
      changed_mode=1;
    }
  } else if (button_status==DEB_TIME) {
    button_time++;
    if (button_time>DEBOUNCE_TIME) {
      button_time=0;
      if (digitalRead(BUTTON)==IS_UNPRESSED) {
        button_status=DEB_TIME2;
      }
    }
  } else if (button_status==DEB_TIME2) {
    button_time++;
    if (button_time>DEBOUNCE_TIME) {
      button_time=0;
      button_status=DEB_NONE;
    }
  }

  if (changed_mode)
  {
    set_mode_led(mode);
  }
}

void manual_debounce(void)
{
  delay(50);
  while(digitalRead(BUTTON)==IS_PRESSED) {
    delay(10);
  }
  delay(50);
}
  

void cal_mode(void)
{
  int i, j, k;
  int maxval, minval;
  minval=0;
  maxval=4500;
  int wait_press=1;
  for (k=0; k<4; k++)
  {
    digitalWrite(led_pin[k], HIGH);
  }
  manual_debounce();
  while(wait_press) {
    for (i=3500; i<4500; i=i+10) {
      meter=i;
      for (j=1; j<200; j++) {
        delay(1);
        if (digitalRead(BUTTON)==IS_PRESSED) {
          maxval=i;
          i=4500;
          j=9999;
          wait_press=0;
          digitalWrite(led_pin[3], LOW);
        }
      }
    }
  }
  manual_debounce();
  wait_press=1;
  while(wait_press) {
    for (i=0; i<200; i=i+10) {
      meter=i;
      for (j=1; j<200; j++) {
        delay(1);
        if (digitalRead(BUTTON)==IS_PRESSED) {
          minval=i;
          i=900;
          j=9999;
          wait_press=0;
          digitalWrite(led_pin[2], LOW);
        }
      }
    }
  }
  manual_debounce();
  // write minval and maxval into EEPROM
  EEPROM.write(0x01, (minval & 0x00ff));
  EEPROM.write(0x02, ((minval>>8) & 0x00ff));
  EEPROM.write(0x03, (maxval & 0x00ff));
  EEPROM.write(0x04, ((maxval>>8) & 0x00ff));
  EEPROM.write(0x00, 0xaa);
  digitalWrite(led_pin[2], HIGH);
  digitalWrite(led_pin[3], HIGH);
  delay(1000);
  set_mode_led(99);
  mmin=minval;
  mmax=maxval;
}

// lose the oldest VOC meter value, and append the latest
void append_vocbuffer(int p)
{
  int i;
  for (i=1; i<VSIZE; i++)
  {
    vocbuffer[i-1]=vocbuffer[i];
  }
  vocbuffer[VSIZE-1]=p;
}

// calculate the moving average
int vocavg(void)
{
  int i;
  int tot=0;
  for (i=0; i<VSIZE; i++) {
    tot=tot+(vocbuffer[i]/VSIZE);
  }
  return tot;
}

// support a min temperature of 15 deg C, and a range of 15 deg C (i.e. max will be 30 deg C)
int temp2pwm(float x)
{
  int p;
  if (x>30.5) x=30.5; // max temperature will be off the scale by 0.5 deg C
  x=x-15.0; // min temperature to be displayed will be 15 deg C
  p=(int)((((float)(mmax-mmin))/15.0)*x);
  p=p+mmin;
  return(p);
}

// support a min RH value of 20%, and a range of 70% (i.e. max will be 90%)
int rh2pwm(float x)
{
  int p;
  if (x>92.0) x=92.0; // max RH will be off the scale by 2%
  x=x-20.0; // min RH to be displayed will be 20%
  p=(int)((((float)(mmax-mmin))/70.0)*x);
  p=p+mmin;
  return(p);
}

// support a VOC range from 1 to 100k ppb on a log scale
// log(1) is 0, log(100k) is 5
int voc2pwm(uint16_t x)
{
  double i;
  int p;
  i=log10((double)x);
  p=(int)((((float)(mmax-mmin))/5.0)*((double)i));
  p=p+mmin;
  return(p);
}

// support a CO2 range from 100 to 100k ppm on a log scale
// log(100) is 2, log(100k) is 5
int co22pwm(uint16_t x)
{
  double i;
  int p;
  i=log10((double)x);
  i=i-2.0; // min to be displayed is 100, i.e. 10^2.0
  p=(int)((((float)(mmax-mmin))/3.0)*((double)i));
  p=p+mmin;
  return(p);
}


// *************** setup code ************************
void setup() {
  int i;

  Serial.begin(9600);
  delay(1000); // let console settle
  
  // Set LEDs as outputs
  for (i=0; i<4; i++)
  {
    pinMode(led_pin[i], OUTPUT);
  }
  pinMode(LED_BOARD, OUTPUT);

  LED_BOARD_OFF;

  // set up button
  pinMode(BUTTON, INPUT_PULLUP);

  // initialize the VOC filter buffer
  for (i=0; i<VSIZE; i++) {
    vocbuffer[i]=mmin;
  }

  // initialize the meter values for each mode
  for (i=0; i<4; i++) {
    mval[i]=mmin;
  }

  // start up PWM mode on the meter output pin
  meter=0;
  DDRB |= 0x20; // set PB5/OC1A pin as output
  ICR1 = 4000;  // freq set for 500Hz. ICR1 = (16M/8)/freq = 2M/freq
  TCCR1A = 0x82; // clear OC1A on compare match
  TCCR1B = 0x1a; // clkio divided by 8, mode 14 waveform: fast PWM, top is ICR1

  // is button pressed at power-on?
  if (digitalRead(BUTTON)==IS_PRESSED) {
    cal_mode();
  } else {
    if (EEPROM.read(0x00)==0xaa)
    {
      i=EEPROM.read(0x02);
      i=i<<8;
      mmin=i|(EEPROM.read(0x01) & 0xff);
      i=EEPROM.read(0x04);
      i=i<<8;
      mmax=i|(EEPROM.read(0x03) & 0xff);
      // sanity
      if ((mmin<0) || (mmin>200)) mmin=100;
      if ((mmax<3000) || (mmax>4500)) mmax=3500;
    }
  }

  // display the cal settings briefly
  meter=mmax;
  delay(3000);
  meter=mmin;
  delay(3000);

  // set up a tick
  TCCR3B = 0x02; // clkio divided by 8
  OCR3A = 2000; // we want an interrupt every millisecond
  TIMSK3 = 0x02; // enable OC3A interrupt

  // light up the mode indication
  set_mode_led(mode);

  // initialize sensors
  ess.initSensors();

}

// ********** main loop called after setup ************
void loop() {
  int i;
  float tf, hf;
  uint16_t vu, cu;
  
  // perform measurements, and convert to a PWM width value
  ess.measureIAQ(); // measure first to ensure proper timing
  ess.measureRHT();
  tf=ess.getTemperature();
  hf=ess.getHumidity();
  vu=ess.getTVOC();
  cu=ess.getECO2();
  mval[0]=temp2pwm(tf);
  mval[1]=rh2pwm(hf);
  mval[2]=voc2pwm(vu);
  mval[3]=co22pwm(cu);
  delay(ess.remainingWaitTimeMS());

  toggle_board_led(); // just as an internal heartbeat indicator

  // if in dynamic view setting, then the mode is updated periodically
  if (update_view)
  {
    update_view=0;
    view_time=0;
    mode++;
    if (mode>3) mode=0;
    set_mode_led(mode);
    if (mode==MODE_VOC) changed_mode=1;
  }

  // update the meter needle
  if (changed_mode)
  {
    changed_mode=0;
    if (mode==MODE_VOC) {
      // set the VOC filter buffer
      for (i=0; i<VSIZE; i++) {
        vocbuffer[i]=mval[mode];
      }
    }
  }

  if (mode==MODE_VOC) {
    append_vocbuffer(mval[mode]);
    meter=vocavg(); 
  } else {
    meter=mval[mode];
  }

  // output the values via serial, as a sanity check
  Serial.print("Temp: ");
  Serial.print(tf);
  Serial.print("Hum: ");
  Serial.print(hf);
  Serial.print("VOC: ");
  Serial.print(vu);
  Serial.print("CO2: ");
  Serial.print(cu);
  Serial.println();
}
