/* !!!! The current code is an old ATtiny code that I will modify for ATmega328p in the coming months !!!! */


#include "SSD1306_minimal.h"
//#include <EEPROM.h>
#include <avr/sleep.h>
#include <math.h>
#include "TC74_I2C.h"

#define Left  3
#define Right 4
#define RF    1
#define Batt  A0

SSD1306_Mini oled;
TC74_I2C TC74(TC74_A5);

volatile uint8_t RF_pulses = 0;
volatile int16_t RPM = 0;
volatile uint8_t adc_value, adc_value_prev = 100;
volatile uint16_t speed_kmh, speed_kmh_prev = 0;
volatile uint16_t current_km_travelled = 22;
float wheel_circumf_m = 2.3;
float wheel_diameter_cm = 74;
float _constant = 0.001885;
float speed_calc_constant = wheel_diameter_cm * _constant;
volatile bool left_button_flag, right_button_flag, both_buttons_flag = false;
volatile byte main_state, timer1_increments = 0;
char string_to_write[5];
volatile bool speed_display_flag = false;
int temperature; 
byte usepowersave = 0;

void sleep() {
  GIMSK |= 1 << PCIE;
  TIMSK |= 1 << OCIE0A;
  ADCSRA |= 0 << ADEN;                    // ADC off
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement
  sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
  sei();
  sleep_cpu();                            // sleep
  cli();
  sleep_disable();                        // Clear SE bit
  ADCSRA |= 1 << ADEN;                    // ADC on
  TIMSK |= 1 << OCIE0A;
  GIMSK |= 1 << PCIE;
  sei();
}

//void scan_I2C(){
//  byte error;
//  char address;
//  int nDevices;
//  for(address = 1; address < 127; address++ )
//  {
//    // The i2c_scanner uses the return value of
//    // the Write.endTransmisstion to see if
//    // a device did acknowledge to the address.
//    Wire.beginTransmission(address);
//    int error = Wire.endTransmission();
// 
//    if (error == 0)
//    {
//      oled.cursorTo(0, 8);
//      oled.printString("0x");
//      if (address<16)
//      oled.cursorTo(10, 8);
//      oled.printString(address);
//      nDevices++;
//    }   
//  }
//}

//void display_speed() {
//  if (speed_display_flag) {
////    calculate_speed();
//    //    if(speed_kmh != speed_kmh_prev){
////    oled.clear();
//    //write_to_display(speed_kmh, 0, 0);
//    //write_to_display(current_km_travelled, 70, 0);
////    write_to_display(TC74.ReadTemp(), 30, 0);
//    speed_kmh_prev = speed_kmh;
//    //    }
//    speed_display_flag = false;
//  }
//}

//void write_to_display(uint16_t data_to_write, int x0, int y0) {
//  itoa(data_to_write, string_to_write, 10);
//  if(data_to_write < 1000){
//    string_to_write[4] = string_to_write[3];
//    string_to_write[3] = string_to_write[2];
//    string_to_write[2] = string_to_write[1];
//    string_to_write[1] = string_to_write[0];
//    string_to_write[0] = ' ';
//  }
//  else if(data_to_write < 100){
//    string_to_write[4] = string_to_write[2];
//    string_to_write[3] = string_to_write[1];
//    string_to_write[2] = string_to_write[0];
//    string_to_write[1] = ' ';
//    string_to_write[0] = ' ';
//  }
//  else if(data_to_write < 10){
//    string_to_write[4] = string_to_write[1];
//    string_to_write[3] = string_to_write[0];
//    string_to_write[2] = ' ';
//    string_to_write[1] = ' ';
//    string_to_write[0] = ' ';
//  }
//  oled.cursorTo(x0, y0);
//  oled.printString(string_to_write);
//}

void calculate_speed() {
//  RPM = RF_pulses * 60;
//  speed_kmh = RPM * speed_calc_constant;
//  RF_pulses = 0;
}

ISR(TIMER0_COMPA_vect) {
  TIFR = 0;
//  timer1_increments++;
//  RF_pulses++;    // for simulating the rf pulses
//  current_km_travelled = (current_km_travelled * 10) + 23;
//  current_km_travelled = current_km_travelled / 10;
//  if (current_km_travelled > 255)
//    current_km_travelled = 255;
//  if (timer1_increments >= 4) {
//    adc_value = analogRead(Batt);
//    speed_display_flag = true;
//    timer1_increments = 0;
//  }
}

void save_mileage_to_mem() {
//  EEPROM.write(1, (8 << current_km_travelled));
//  EEPROM.write(2, current_km_travelled);
}

void recover_mileage_from_mem() {
//  current_km_travelled = (8 << EEPROM.read(1)) + EEPROM.read(2);
}

ISR(PCINT0_vect)
{
  if (!digitalRead(RF))
  {
    RF_pulses++;             // Increment volatile variable
    current_km_travelled += wheel_circumf_m;
    //    if(RF_pulses > 32767)
    //    {
    //      RF_pulses = 32767;
    //    }
  }
  if (!digitalRead(Left))
  {
    left_button_flag = true;
  }
  if (!digitalRead(Right))
  {
    right_button_flag = true;
  }
  if (right_button_flag && left_button_flag) {
    both_buttons_flag = true;
  }
}

void setup() {
  pinMode(RF, INPUT_PULLUP);
  pinMode(Batt, INPUT);
  pinMode(Left, INPUT);
  pinMode(Right, INPUT_PULLUP);

  TCCR0B = 0;
  TCCR0B = 0x05;
  TCCR0A = 0x02;
  OCR0A = 243;

//  scan_I2C();
  oled.init(0x3c);
  oled.clear();

  TIMSK = 1 << OCIE0A;
  GIMSK |= 1 << PCIE;    // turns on pin change interrupts
  PCMSK = 0b11010;    // turn on interrupts on pins PB1, PB3 and PB4
//  sei();
  
  oled.startScreen();
  oled.cursorTo(0, 0);
  oled.printString("Start");
  delay(500);
  oled.clear();
  TC74.Init();
  if(!TC74.Powersavestatus())
  {
    oled.cursorTo(0, 8);
    oled.printString("No TC74");
  }
  else
  {
    oled.cursorTo(0, 8);
    oled.printString("Found TC74");
  }
}

void loop() {
//  display_speed();
  // sleep();

  //  delay(1000);
}



