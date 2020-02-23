/*
Arduino PressureController2OS
 
  This is an Arduino code that operates two valves in order to control the pressure
 in a closed volume. One valve is connected to a pressure supply source ("supply" or "vac")
 and the other is connected to atmosphere ("vent"). The controller is a PID controller.
  This program is meant to be used in conjuction with a user interface running on a computer.

Matt Hopcroft, Red Dog Research
 hopcroft@reddogresearch.com

Copyright 2020

This code is distributed under the GPLv3 license. It includes independent libraries which
  are distributed under their own licenses. See the links below for documentation and sources
  for these libraries. This code can be built and installed using the Arduino IDE and the
  usual procedures for programming Arduinos.

  The GNU General Public License: http://www.gnu.org/licenses/gpl.html

  External Libraries:
  EEPROMex (LGPL license): https://github.com/thijse/Arduino-EEPROMEx
  Arduino-PID-Library (GPLv3 license): https://github.com/br3ttb/Arduino-PID-Library


*/

// Additional libraries. Must download/install these libraries in the Arduino IDE
#include <EEPROMex.h>
#include <PID_v1.h>

// Output Pins
//  Use pins 3,11 for PWM output
//   Code for setting the PWM output frequency is below
#define VENT 11
#define VACUUM 3
#define LED_onboard 13
#define LED_status 12

//Input Pins
// Read analog pressure sensor on pin 1
//  the pressure sensor gives an analog voltage output 0.5-4.5V
#define SensorIn 1

// firmware version
char verstring[25] = "PressureController2 OSv2";
int vernum = 2; // match above

String inputString = "";         // a string to hold incoming commands
boolean stringComplete = false;  // flag for command string is complete
long baudrate = 115200;           // initial baudrate

// Variables for the pressure sensor
//  Assuming use of pressure sensor Honeywell ASDX015PDAA5, with range +/- 15 psi (103 kPa)
//  Arduino analog inputs have 10-bit resolution
int sensorZero = 512; // sensor signal reads this value at 0 input (eg ATM for diff. sensor)
double sensorPressure = sensorZero; // reading from the sensor
double currentPressure = sensorZero; // filtered/conditioned pressure value

// for exponential filter
int cutoff = 0;  // cutoff (3db) frequency for LP filter (Hz). Use 0 to disable
const double dt = 0.004160;  // sampling interval in seconds. Interrupt for sampling set at 240 Hz below.
double tau = 1/(2*PI*cutoff);
double K = dt / (tau + dt);

// for moving average filter
// http://dsp.stackexchange.com/questions/9966/what-is-the-cut-off-frequency-of-a-moving-average-filter
byte numReadings = 8;      // N=8 @ 240 Hz to hit 30,60,90 Hz etc
int readings[21];         // the readings from the analog input (N=21 max)
byte index_p = 0;          // the index of the current reading
double totalPressure = 0; // the running total

// Valve control
// The PWM outputs are 8-bit
const byte VOPEN = 255;
const byte VCLOSED = 0;
boolean cmdControl = false;
boolean setControl = false;
boolean exceededErr = false; // has an error condition occurred?
int errorThrshld = 0; // use 0 to disable error check (or 1024)
int errorVal = 0; // the error value that triggered the warning
int errorMax = 0; // maximum error value
// Valve control output
double PID_ctrl;  // Result from PID calculation
byte VA_Out, VE_Out; // 8-bit PWM output value
// valve operation parameters
byte veMin = 75;
byte veSpan = 100;
byte vaMin = 95;
byte vaSpan = 120;

// PID parameters
double targetPressure = sensorZero;
// define a struct for PID gains
typedef struct {
  double Kp = 1;
  double Ki = 1;
  double Kd = 0;
} PIDGains;
PIDGains pid_hold, pid_track;
int sampleTime = 10;  // PID evaluation time in ms
// Use PID library - create PID object
// Syntax:   PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
//  Note: All inputs are type double (4 bytes)
PID valvePID(&currentPressure,&PID_ctrl,&targetPressure,pid_hold.Kp,pid_hold.Ki,pid_hold.Kd,DIRECT);

// arrays for holding the program steps
//  Each "step" is a type of waveform for pressure control with the required parameters
//  Type, Amplitude, Offset, Frequency, Duty Cycle, Time of Step
int index_cmd = 0;    // current programmed waveform that is executing
int index_prog = 0;   // next free programming waveform slot
const byte max_waveforms = 10;
char wType[max_waveforms];          // the Type of the waveform (constant, sin, etc)
int wAmp[max_waveforms];            // the Amplitude
int wOffset[max_waveforms];         // the Offset
double wFreq[max_waveforms];        // the Frequency
unsigned long wPer[max_waveforms];  // the Period
int wDuty[max_waveforms];           // the Duty Cycle
unsigned long wTime[max_waveforms]; // the execution Time
boolean runList = false;  // are we executing the list or not
int numRepeat = 0; // how many times has the waveform list repeated
int waveRpt = 0; // how many times is the waveform list allowed to repeat

// EEPROM addresses (memory map) in bytes offset from zero.
//     1024 bytes total on Atmega328 (Uno, Duemilanove)
const int ver_addr = 0;
const int pid1_addr = 2; // only 12 bytes each, leave some room to expand
const int pid2_addr = 34;
const int filterN_addr = 66;  // int numReadings
const int filterLP_addr = 68; // int cutoff
const int sensorZero_addr = 70; // int sensorZero
const int sampleTime_addr = 72; // int sampleTime (for PID controller)
const int valveMinMax_addr = 78; // four bytes: vaMin, vaSpan, veMin, veSpan

// time keeping
unsigned long dTime = 0;
//double lTime = 0;
unsigned long time_prev; // timing for data output;
unsigned long time_wave, time_cmd; // timing for waveform list;
unsigned long time_err = 0;
unsigned long time_errMax = 0;

// functions
int sendData = 0;               // continuously transmit data to USB host
int makeData = 5;               // timer for data generation, default 5 ms
int dataInterval = 50;          // transmit data periodically, default 50 ms = 20 Hz
int printData = dataInterval;   // timer for data transmission
boolean debug = true;           // print debug messages
boolean debugPressure = false;   // print pressure sensor readings once per sec

// // // //



////////////////////
//timer1 interrupt service routine
// gets reading from the pressure sensor and applies filtering
ISR(TIMER1_COMPA_vect){   
// Get the value from the pressure sensor
  sensorPressure = analogRead(SensorIn);

  // two filter options: exponential or moving average (or both)
  
  if (cutoff > 0) { // use exponential lowpass filter, with K = dt / (tau + dt)
    currentPressure = currentPressure + K * (sensorPressure - currentPressure);
  } else {
    currentPressure = sensorPressure;
  }
  
  if (numReadings > 0) {  // use a moving average filter
    // 1) remove the oldest reading:
    totalPressure = totalPressure - readings[index_p];         
    // 2) get sensor reading:
    readings[index_p] = currentPressure; 
    // 3) add the reading to the total:
    totalPressure = totalPressure + readings[index_p];       
    // 4) advance to the next position in the array or reset the count:  
    index_p++;
    if (index_p >= numReadings) index_p = 0;
    // 5) calculate the moving average:
    currentPressure = totalPressure / numReadings;
  } 

}




////////////////////
// put setup code here, to run once:
void setup() {

  noInterrupts();  //disable interrupts during setup [ same as cli() ]
  
  // initialize digital pins as an output, for onboard LED:
  pinMode(LED_onboard, OUTPUT);
  pinMode(LED_status, OUTPUT);

  // get saved values from EEPROM
  //  check version number. If new version, [re]initialize the saved values to default
  if ( EEPROM.updateBlock(ver_addr,vernum) == 0 ) { // normal case
    EEPROM.readBlock(pid1_addr,pid_hold);
    EEPROM.readBlock(pid2_addr,pid_track);
    numReadings = EEPROM.readInt(filterN_addr);
    cutoff = EEPROM.readInt(filterLP_addr);
    // calculate the LP filter values
    tau = 1/(2*PI*cutoff);
    K = dt / (tau + dt);  // dt set for 240 Hz sampling
    sensorZero = EEPROM.readInt(sensorZero_addr);
    sampleTime = EEPROM.readInt(sampleTime_addr);
    vaMin = EEPROM.readByte(valveMinMax_addr);
    vaSpan = EEPROM.readByte(valveMinMax_addr+1);
    veMin = EEPROM.readByte(valveMinMax_addr+2);
    veSpan = EEPROM.readByte(valveMinMax_addr+3);
    
    
  } else {  // occurs first time the controller is used
    blinker(LED_onboard, 2);
    EEPROM.updateBlock(pid1_addr,pid_hold);
    EEPROM.updateBlock(pid2_addr,pid_track);
    EEPROM.updateInt(filterN_addr,numReadings);
    EEPROM.updateInt(filterLP_addr,cutoff);
    EEPROM.updateInt(sensorZero_addr,sensorZero);
    EEPROM.updateInt(sampleTime_addr,sampleTime);
    EEPROM.updateByte(valveMinMax_addr,vaMin);
    EEPROM.updateByte(valveMinMax_addr+1,vaSpan);
    EEPROM.updateByte(valveMinMax_addr+2,veMin);
    EEPROM.updateByte(valveMinMax_addr+3,veSpan);
  }
  
  // set the PID controller with the saved values
  valvePID.SetTunings(pid_hold.Kp, pid_hold.Ki, pid_hold.Kd);

  // initialize the control pins as outputs for PWM
  // set the pwm frequency for pins 3,11 to 1 kHz using Timer2
  TCCR2B = TCCR2B & B11111000 | B00000011;    // set timer 2 divisor to 32 for PWM frequency of 980.39 Hz
  pinMode(VENT, OUTPUT);
  pinMode(VACUUM, OUTPUT);
  // initialize both valves to closed position
  analogWrite(VACUUM, VCLOSED);
  analogWrite(VENT, VCLOSED);

  // initialize the list of waveforms
  initWaveforms(max_waveforms, &index_prog, &index_cmd, wType, wAmp, wOffset, wPer, wFreq, wDuty, wTime);

  // initialize array of pressure sensor readings to 0 for moving average: 
  for (int thisReading = 0; thisReading < 21; thisReading++)
    readings[thisReading] = sensorZero;
  totalPressure = sensorZero * numReadings;

  // Configure PID
  valvePID.SetSampleTime(sampleTime);       // execute every X ms
  valvePID.SetOutputLimits(0,1);            // control signal limits

  // configure Timer1 to read the pressure sensor at at regular interval
  // http://www.instructables.com/id/Arduino-Timer-Interrupts/step2/Structuring-Timer-Interrupts/
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for sample frequency (Hz)
  //  match value = (16*10^6) / (prescale * freq) - 1 (must be <65536)
  //OCR1A = 63999;  //use 63999,1 for 250 Hz 
  OCR1A = 64;   // use 64,1024 for 240 Hz
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 bit for 1 (none) prescaler
  //TCCR1B |= (1 << CS10);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  // set up Serial comm with standard settings
  Serial.begin(baudrate,SERIAL_8N1);
  Serial.flush();

  interrupts(); //allow interrupts [ same as sei() ]
  
  // blink the LED, to show end of init
  blinker(LED_status, 3);  
  if (debug == true) {
    blinker(LED_onboard, 3);
  }
  
  // init setpoint to zero
  targetPressure = sensorZero; 
  
  //Serial.println(verstring);
  time_prev = millis(); // timer for data streaming
  //delay(100);

} // end setup



////////////
// put the main code here, to run repeatedly:
void loop() {

  // record time each time loop() iterates
  unsigned long time_run = millis();
  unsigned long time_loop = time_run - time_prev;
  time_prev = time_run;

//  if (runList == false) {
//    time_wave = time_run; // time for the current waveform step
//  }

  ////////
  // Execute waveform program if runList is true
  // Compute target pressure for the current waveform program (const, sin, etc)
  //  Each waveform step is executed for time_cmd ms, then move to the next waveform
  if (runList == true) {
    time_cmd = time_run - time_wave;
    if (wTime[index_cmd] == 0 || time_cmd <= wTime[index_cmd]) {
      // determine the target pressure based on function and time
      switch (wType[index_cmd]) {
        case 'c':
          targetPressure = wOffset[index_cmd];
          break;
        case 's':
          targetPressure = wAmp[index_cmd]*(sin(2*PI*wFreq[index_cmd]*(double)time_cmd/1000.0)) + wOffset[index_cmd];
          break;
        case 'q':
          targetPressure = squareWave(wPer[index_cmd], wAmp[index_cmd], wOffset[index_cmd], wDuty[index_cmd], time_cmd);
          break;
        case 'r':
          targetPressure = rampWave(wPer[index_cmd], wAmp[index_cmd], wOffset[index_cmd], wDuty[index_cmd], time_cmd);
          break;
        case 'x':
          Serial.println("Type x in list");
          runList = false;  // stop executing list
          break;
      }
    } else { // advance to next waveform in list
      index_cmd++;
      time_wave = millis(); // time_wave is start time of each waveform
      // waveform status output
      if (debug == true) {
        //Serial.println(";"); // to interrupt data output
        Serial.print("WAVE"); Serial.println(index_cmd,DEC);
      }
      
      if (index_cmd >= index_prog) { // all waveforms have been executed
        index_cmd = 0;
        // v28+: at end of waveform list, restart list instead of stopping
        numRepeat++; // counter for waveform repeats
        if (debug == true) {
          Serial.print("End waveform list ("); Serial.print(numRepeat, DEC); Serial.println(")");
        }
      }
      
      // go to next waveform in list
      // set PID parameters according to waveform type
      if (wType[index_cmd] == 'c' || wType[index_cmd] == 'q') {
        valvePID.SetTunings(pid_hold.Kp, pid_hold.Ki, pid_hold.Kd);
      } else {
        valvePID.SetTunings(pid_track.Kp, pid_track.Ki, pid_track.Kd);
      }

      // waveform list has repeated the specified number of times
      if (waveRpt > 0 && numRepeat >= waveRpt) { // NB these values are bytes
        // stop waveforms
        Serial.println(";"); // to interrupt data output
        Serial.println("WSTP");
        runList = false;
        // go to "hold" settings in PID
        valvePID.SetTunings(pid_hold.Kp, pid_hold.Ki, pid_hold.Kd);
      }
      
      if (debug == true) {
        blinker(LED_onboard, index_cmd+1); // NB this takes significant time
      }
 
    } // end advance waveform index
    
    if (debug == true && time_run % 1000 < 5) {
      Serial.print("Target: ");
      Serial.print(targetPressure, DEC);
      Serial.print(" Time: ");
      Serial.print(time_cmd, DEC);
      Serial.print(" / ");
      Serial.print(wTime[index_cmd], DEC);
      Serial.print(" Type: ");
      Serial.print(wType[index_cmd]);      
      Serial.print(" IndexCmd: ");
      Serial.println(index_cmd, DEC);
      }
  }



  ///////////
  // apply the PID control algorithm
  //noInterrupts(); // in theory, the value of currentPressure might be changed by interrupt. No problems observed...
  setControl = valvePID.Compute();
  //interrupts();
  // PID sets value of PID_ctrl
  if (setControl == true) {

    // this function converts a "control value" of 0-1.0 (0-100 %)
    //  to the specific valve PWM outputs
    applyControl(&PID_ctrl, &VA_Out, &VE_Out);

    // check for error condition
    int ctrlerror = (int)(currentPressure - targetPressure);
    if ( errorThrshld > 0 && abs(ctrlerror) > errorThrshld ) {
      // record first error condition
      if (exceededErr == false) {
        errorVal = (int)currentPressure;
        time_err = millis();
        exceededErr = true;
      }
      // record maximum error
      if (abs(ctrlerror) > abs(errorMax)) {
        errorMax = (int)ctrlerror;
        time_errMax = millis(); 
      }
    }
  }

  // and light the status LED if error
  if (exceededErr == true) {
    digitalWrite(LED_status, HIGH);
    if (debug == true) digitalWrite(LED_onboard, HIGH);
  }
  // reset status light
  if (exceededErr == false) {
    digitalWrite(LED_status, LOW);
    if (debug == true) digitalWrite(LED_onboard, LOW);
  }


  /////////////////////
  // Send the results to the host computer
  //  string format: (max 64 bytes incl LF)
  //   $time stamp,pressure sensor,target pressure,PID_value,vacuum_out,vent_out
  //  send rate of 10ms with baud rate of 57600 typically ok
  // v22: generate data at one rate, and trigger host to processor data at another (slower) rate
  // Note: the "comparison timer" is more reliable than the "mod timer"
  //   See: http://www.forward.com.au/pfod/ArduinoProgramming/TimingDelaysInArduino.html

  // generate data strings at makeData intervals
  if (sendData > 0) makeData -= time_loop;
  //Serial.println(makeData, DEC);
  if (sendData > 0 && makeData <= 0 ) {
    String output_string = String( "$" + String(time_run) + "," + String(currentPressure) + "," + String(targetPressure) + "," + String(PID_ctrl) + "," + String(VA_Out) + "," + String(VE_Out) );
    Serial.print(output_string); // Serial.print(" : ");
    makeData += sendData;
  }
  // send a line terminator at printData intervals
  if (sendData > 0) printData -= time_loop;
  //Serial.println(printData, DEC);
  if (sendData > 0 && printData <= 0 ) {
    Serial.println(";"); // the host does not process data until it receives the line terminator (cr/lf)
    printData += dataInterval;
  }

  
  /////////////////
  // If a command string has been received from the host computer,
  //  process the command
  if (stringComplete) {
    Serial.println(";"); // this separates commands from the data stream
    
    if (debug == true) {
      Serial.println("RECV " + inputString);
      blinker(LED_onboard, 1);   // blink led
      blinker(LED_status, 1);   // blink led
    }


    if (inputString.startsWith("vr")) {   // send version info
      Serial.println(verstring);
    }

    
    else if (inputString.startsWith("bk")) {   // blink the onboard LED
      if (debug == true) {
        blinker(LED_onboard, inputString.substring(2).toInt());   // blink led
      }
      blinker(LED_status, inputString.substring(2).toInt());   // blink led
      Serial.println("OK");
    }


    else if (inputString.startsWith("st")) {   // return status information
      Serial.print("Pressure Control: "); Serial.println(valvePID.GetMode());
      Serial.print("Current Pressure: "); Serial.print(currentPressure, 2); Serial.println(" counts");
      Serial.print("Target Pressure: "); Serial.print(targetPressure, 2); Serial.println(" counts");
      Serial.print("Waveform List Run: "); Serial.println(runList);
      Serial.print("Current Waveform: "); Serial.print(index_cmd+1, DEC); Serial.print(" of "); Serial.println(index_prog, DEC);
      Serial.print("Waveform Repeat: "); Serial.print(numRepeat, DEC); Serial.print(" of "); Serial.println(waveRpt, DEC);
      Serial.print("Run Time: "); Serial.print(time_run, DEC); Serial.println(" ms");
      Serial.print("Sensor exp. LP cutoff: "); Serial.print(cutoff, DEC); Serial.println(" Hz");
      Serial.print("Sensor moving average N: "); Serial.println(numReadings, DEC);
      Serial.print("Sensor Zero value: "); Serial.println(sensorZero, DEC);
      Serial.print("PID Interval: "); Serial.print(sampleTime, DEC); Serial.println(" ms");
      Serial.print("PID parameters (working): "); Serial.print(valvePID.GetKp(), DEC); Serial.print(", "); Serial.print(valvePID.GetKi(), DEC); Serial.print(", "); Serial.println(valvePID.GetKd(), DEC);
      Serial.print("PID parameters (hold): "); Serial.print(pid_hold.Kp, DEC); Serial.print(","); Serial.print(pid_hold.Ki, DEC); Serial.print(","); Serial.println(pid_hold.Kd, DEC);
      Serial.print("PID parameters (track): "); Serial.print(pid_track.Kp, DEC); Serial.print(","); Serial.print(pid_track.Ki, DEC); Serial.print(","); Serial.println(pid_track.Kd, DEC);
      Serial.print("Control Error Threshold: "); Serial.print(errorThrshld, DEC); Serial.println(" counts");
      Serial.print("Controller Output: "); Serial.print((PID_ctrl*100), 1); Serial.println(" %");
      Serial.print("Supply Valve Setting: "); Serial.println(VA_Out, DEC);
      Serial.print("Vent Valve Setting:   "); Serial.println(VE_Out, DEC);
      Serial.print("Valve Parameters: "); Serial.print(vaMin,DEC); Serial.print(","); Serial.print(vaSpan,DEC); Serial.print(","); Serial.print(veMin,DEC); Serial.print(","); Serial.println(veSpan,DEC);
      Serial.print("Valve Compensation: "); Serial.print(vlvcomp[0],DEC); Serial.print(","); Serial.print(vlvcomp[1],DEC); Serial.print(","); Serial.println(vlvcomp[2],DEC);
      Serial.print("Sending Data: "); Serial.println(sendData);
      Serial.print("Data Interval: "); Serial.print(dataInterval); Serial.println(" ms");
      Serial.print("Debug mode: "); Serial.println(debug);
      Serial.println("OK");
    }

    else if (inputString.startsWith("db")) {   // enable/disable debug mode
      if (inputString.length() == 2) {
         Serial.print("db:"); Serial.println(debug,DEC);
      } else if (inputString[2] == '0') {
        debug = false;
        Serial.println("OK");
      } else if (inputString[2] == '1') {
        debug = true;
        Serial.println("OK");
      }
    }

    else if (inputString.startsWith("br")) {   // set baudrate
      if (inputString.length() == 2) {
        Serial.print("br:"); Serial.println(baudrate,DEC);
      } else {
        if (inputString.substring(2,inputString.length()).toInt() > 0) {
          float brtemp = inputString.substring(2,inputString.length()).toFloat();
          baudrate = (long)brtemp;
          Serial.print("br:"); Serial.println(baudrate,DEC);
          Serial.println("OK");
          Serial.end();
          delay(1000);
          Serial.begin(baudrate,SERIAL_8N1);
          blinker(LED_status, 2);   // blink led
          delay(200);
          blinker(LED_status, 2);
        } else {
          Serial.println("?");
        }
      }
    }   

    else if (inputString.startsWith("ct")) {   // enable/disable controlling pressure
      if (inputString.length() == 2) {
         Serial.print("ct:"); Serial.println(cmdControl,DEC);
      } else if (inputString[2] == '0') {
        cmdControl = false;
        valvePID.SetMode(MANUAL);
        Serial.println("OK");
      } else if (inputString[2] == '1') {
        cmdControl = true;
        valvePID.SetMode(AUTOMATIC);
        Serial.println("OK");
      }
    }

    else if (inputString.startsWith("tg")) {   // set target pressure (setpoint)
      if (inputString.length() == 2) {
        Serial.print("tg:"); Serial.println(targetPressure,DEC);
      } else {
        targetPressure = inputString.substring(2,inputString.length()).toFloat(); // should this be toFloat?
        Serial.println("OK");
      }
    }

    else if (inputString.startsWith("em")) {   // set control error max warning threshold
      if (inputString.length() == 2) {
        Serial.print("em:"); Serial.println(errorThrshld,DEC);
      } else {
        errorThrshld = inputString.substring(2,inputString.length()).toInt();
        Serial.println("OK");
      }
    }

    else if (inputString.startsWith("te")) {   // get time of control error, or reset warning
      if (inputString.length() == 2) {
        Serial.print("te:"); Serial.print(time_err,DEC); Serial.print(","); Serial.print(millis(),DEC); Serial.print(","); Serial.print(errorVal,DEC);
        Serial.print(","); Serial.print(time_errMax,DEC); Serial.print(","); Serial.println(errorMax,DEC);
      } else if (inputString[2] == '0') { // reset error warning
        time_err = 0;
        errorVal = 0;
        exceededErr = false;
        digitalWrite(LED_status, LOW);
        Serial.println("OK");
      }
    }
        
    else if (inputString.startsWith("rn")) {   // enable/disable executing waveform list
      if (inputString.length() == 2) {
         Serial.print("rn:"); Serial.println(runList,DEC);
      } else if (inputString[2] == '0') { // stop executing list
        runList = false;
        // go to "hold" PID parameters
        valvePID.SetTunings(pid_hold.Kp, pid_hold.Ki, pid_hold.Kd);
        Serial.println("OK");
      } else if (inputString[2] == '1') { // start executing list
        // set PID parameters according to waveform type
        if (wType[index_cmd] == 'c') {
          valvePID.SetTunings(pid_hold.Kp, pid_hold.Ki, pid_hold.Kd);
        } else {
          valvePID.SetTunings(pid_track.Kp, pid_track.Ki, pid_track.Kd);
        }
        runList = true;
        cmdControl = true;        
        valvePID.SetMode(AUTOMATIC);
        exceededErr = false;
        Serial.println("OK");
      }
    }
    

    
    else if (inputString.startsWith("w")) {   // command for adding a waveform (wc, ws, etc)
      if (index_prog < max_waveforms) {
        wType[index_prog] = inputString[1];
        parseWave(inputString.substring(2,inputString.length()), &wPer[index_prog], &wFreq[index_prog], &wAmp[index_prog], &wOffset[index_prog], &wDuty[index_prog], &wTime[index_prog]);
        // set the PID coefficients for the first waveform in the list
        if (index_prog == 0) {
          // set PID parameters according to waveform type
          if (wType[index_prog] == 'c') {
            valvePID.SetTunings(pid_hold.Kp, pid_hold.Ki, pid_hold.Kd);
          } else {
            valvePID.SetTunings(pid_track.Kp, pid_track.Ki, pid_track.Kd);
          }          
        }
        index_prog++;
        Serial.println("OK");

      } else {
        Serial.println("?");
      }
    }
 
    else if (inputString.startsWith("ci")) { // select which waveform to execute
      if (inputString.length() == 2) {
        Serial.print("ci:"); Serial.println(index_cmd, DEC);
      } else {
        index_cmd = inputString.substring(2,inputString.length()).toInt();
        if (index_cmd >= index_prog) {
          Serial.println("?");
        } else {
          // set PID parameters according to waveform type
          if (wType[index_cmd] == 'c') {
            valvePID.SetTunings(pid_hold.Kp, pid_hold.Ki, pid_hold.Kd);
          } else {
            valvePID.SetTunings(pid_track.Kp, pid_track.Ki, pid_track.Kd);
          }
          time_wave = millis();
          Serial.println("OK");
        }
      }
    }
    
    else if (inputString.startsWith("cw")) {   // clear waveform program list
      initWaveforms(max_waveforms, &index_prog, &index_cmd, wType, wAmp, wOffset, wPer, wFreq, wDuty, wTime);
      index_cmd = 0;
      Serial.println("OK");
    }

    else if (inputString.startsWith("rs")) {   // restart waveform program list
      index_cmd = 0;
      time_wave = millis();
      runList = true;
      cmdControl = true;
      valvePID.SetMode(AUTOMATIC);
      Serial.println("OK");
    }

    else if (inputString.startsWith("gp")) {   // send current pressure measurement
      Serial.print("gp:"); Serial.print(millis(), DEC);Serial.print(","); Serial.println(currentPressure, DEC);
    }
        
    else if (inputString.startsWith("pd")) {   // get current PID parameters
      if (inputString.length() == 2) {
        Serial.print("pd:");
        Serial.print(valvePID.GetKp(), DEC); Serial.print(", "); Serial.print(valvePID.GetKi(), DEC); Serial.print(", "); Serial.println(valvePID.GetKd(), DEC);
      } else {
        Serial.println("?");
      }
    }
    
    else if (inputString.startsWith("ph")) {   // set PID parameters (hold)
      if (inputString.length() == 2) {
        Serial.print("ph:");
        Serial.print(pid_hold.Kp, DEC); Serial.print(","); Serial.print(pid_hold.Ki, DEC); Serial.print(","); Serial.println(pid_hold.Kd, DEC);
      } else {
        
        String cmdstr = inputString.substring(2,inputString.length());
        String arg;
        int beginIdx = 0;
        int idx = 0;
        
        idx = cmdstr.indexOf(",",beginIdx);
        arg = cmdstr.substring(beginIdx, idx);
        pid_hold.Kp = arg.toFloat();
        beginIdx = idx + 1;

        idx = cmdstr.indexOf(",",beginIdx);
        arg = cmdstr.substring(beginIdx, idx);
        pid_hold.Ki = arg.toFloat();
        beginIdx = idx + 1;

        idx = cmdstr.indexOf(",",beginIdx);
        arg = cmdstr.substring(beginIdx, idx);
        pid_hold.Kd = arg.toFloat();
        beginIdx = idx + 1;

        // set the controller to this setting
        valvePID.SetTunings(pid_hold.Kp, pid_hold.Ki, pid_hold.Kd);

        Serial.println("OK");

      }
    }

    else if (inputString.startsWith("pt")) {   // set PID parameters (track)
      if (inputString.length() == 2) {
        Serial.print("pt:");
        Serial.print(pid_track.Kp, DEC); Serial.print(","); Serial.print(pid_track.Ki, DEC); Serial.print(","); Serial.println(pid_track.Kd, DEC);
      } else {
        
        String cmdstr = inputString.substring(2,inputString.length());
        String arg;
        int beginIdx = 0;
        int idx = 0;        
        
        idx = cmdstr.indexOf(",",beginIdx);
        arg = cmdstr.substring(beginIdx, idx);
        pid_track.Kp = arg.toFloat();
        beginIdx = idx + 1;

        idx = cmdstr.indexOf(",",beginIdx);
        arg = cmdstr.substring(beginIdx, idx);
        pid_track.Ki = arg.toFloat();
        beginIdx = idx + 1;

        idx = cmdstr.indexOf(",",beginIdx);
        arg = cmdstr.substring(beginIdx, idx);
        pid_track.Kd = arg.toFloat();
        beginIdx = idx + 1;

        // set the controller to this setting
        valvePID.SetTunings(pid_track.Kp, pid_track.Ki, pid_track.Kd);

        Serial.println("OK");

      }
    }

    else if (inputString.startsWith("si")) {   // set PID sample interval
      if (inputString.length() == 2) {
        Serial.print("si:"); Serial.println(sampleTime,DEC);
      } else {
        sampleTime = inputString.substring(2,inputString.length()).toInt();
        valvePID.SetSampleTime(sampleTime);
        Serial.println("OK");
      }
    }
        
    else if (inputString.startsWith("mv")) {   // use moving average filter on pressure sensor signal
      if (inputString.length() == 2) {
        Serial.print("mv:"); Serial.println(numReadings,DEC);
      } else {
        numReadings = inputString.substring(2,inputString.length()).toInt();
        if (numReadings > 21) {
          Serial.println("?");
        } else {
          cli();  //disable interrupts
          totalPressure = currentPressure * numReadings;
          index_p = 0;
          readings[index_p] = currentPressure;
          Serial.println("OK");
          sei();  //allow interrupts
        }
      }
    }

    else if (inputString.startsWith("lp")) {   // use exponential LP filter on pressure sensor signal
      if (inputString.length() == 2) {
        Serial.print("lp:"); Serial.println(cutoff,DEC);
      } else {
        cli();  //disable interrupts
        cutoff = inputString.substring(2,inputString.length()).toFloat();
        tau = 1/(2*PI*cutoff);
        K = dt / (tau + dt);  // dt set for 240 Hz sampling
        Serial.println("OK");
        sei();  //allow interrupts
      }
    }
  
    else if (inputString.startsWith("va")) {   // "manually" set vacuum valve position
      if (inputString.length() == 2) {
         Serial.print("va:"); Serial.println(VA_Out,DEC);
      } else {
        VA_Out = inputString.substring(2,inputString.length()).toInt();
        analogWrite(VACUUM, VA_Out);
        Serial.println("OK");
      }
    }
    
    else if (inputString.startsWith("ve")) {   // "manually" set vent valve position
      if (inputString.length() == 2) {
         Serial.print("ve:"); Serial.println(VE_Out,DEC);
      } else {
        VE_Out = inputString.substring(2,inputString.length()).toInt();
        analogWrite(VENT, VE_Out);
        Serial.println("OK");
      }
    }

    else if (inputString.startsWith("vs")) {   // set valve positions using control signal
      if (inputString.length() == 2) {
         Serial.print("vs:"); Serial.println(PID_ctrl,2);
      } else {
        PID_ctrl = inputString.substring(2,inputString.length()).toFloat();
        if (PID_ctrl > 1) PID_ctrl = PID_ctrl/100;
        if (PID_ctrl >= 0 && PID_ctrl <= 1) {
          applyControl(&PID_ctrl, &VA_Out, &VE_Out);
          Serial.println("OK");
        } else {
          Serial.println("?");
        }
      }
    }
    
    else if (inputString.startsWith("cl")) {  // close both valves
      VE_Out = VCLOSED;
      VA_Out = VCLOSED;
      analogWrite(VACUUM, VA_Out);
      analogWrite(VENT, VE_Out);
      Serial.println("OK");
    }

    else if (inputString.startsWith("op")) {  // open both valves
      VE_Out = VOPEN;
      VA_Out = VOPEN;
      analogWrite(VACUUM, VA_Out);
      analogWrite(VENT, VE_Out);
      Serial.println("OK");
    }

    else if (inputString.startsWith("vp")) {   // set valve min/max parameters
      if (inputString.length() == 2) {
         Serial.print("vp:"); Serial.print(vaMin,DEC); Serial.print(","); Serial.print(vaSpan,DEC); Serial.print(","); Serial.print(veMin,DEC); Serial.print(","); Serial.println(veSpan,DEC);  
      } else {
        
        String cmdstr = inputString.substring(2,inputString.length());
        String arg;
        int beginIdx = 0;
        int idx = 0;        
        
        idx = cmdstr.indexOf(",",beginIdx);
        arg = cmdstr.substring(beginIdx, idx);
        vaMin = byte(arg.toInt());
        beginIdx = idx + 1;

        idx = cmdstr.indexOf(",",beginIdx);
        arg = cmdstr.substring(beginIdx, idx);
        vaSpan = byte(arg.toInt());
        beginIdx = idx + 1;

        idx = cmdstr.indexOf(",",beginIdx);
        arg = cmdstr.substring(beginIdx, idx);
        veMin = byte(arg.toInt());
        beginIdx = idx + 1;

        idx = cmdstr.indexOf(",",beginIdx);
        arg = cmdstr.substring(beginIdx, idx);
        veSpan = byte(arg.toInt());
        beginIdx = idx + 1;

        //vaConv2pwm = (vaSpan - vaMin)/100;
        //veConv2pwm = (veSpan - veMin)/100;

        Serial.println("OK");

      }
    }    

    else if (inputString.startsWith("sz")) {  // sensor zero value
      if (inputString.length() == 2) {
         Serial.print("sz:"); Serial.println(sensorZero,DEC);
      } else {
        sensorZero = inputString.substring(2,inputString.length()).toInt();
        Serial.println("OK");
      }
    }
        
    else if (inputString.startsWith("sv")) {  // save settings to EEPROM
      noInterrupts();
      EEPROM.updateBlock(pid1_addr,pid_hold);
      EEPROM.updateBlock(pid2_addr,pid_track);
      EEPROM.updateInt(filterN_addr,numReadings);
      EEPROM.updateInt(filterLP_addr,cutoff);
      EEPROM.updateInt(sensorZero_addr,sensorZero);
      EEPROM.updateInt(sampleTime_addr,sampleTime);
      EEPROM.updateByte(valveMinMax_addr,vaMin);
      EEPROM.updateByte(valveMinMax_addr+1,vaSpan);
      EEPROM.updateByte(valveMinMax_addr+2,veMin);
      EEPROM.updateByte(valveMinMax_addr+3,veSpan);
      
      Serial.println("OK");
      interrupts();
    }
    
    else {    // command not understood
      Serial.println("?");
    }
    
    // clear the input string:
    inputString = "";
    stringComplete = false;
    
  }

  // End command processing


} // end main loop




/*****************
  Sub-functions
*/

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX. This routine is run between each
 time loop() runs, so using delay inside loop can delay
 uC response. Incoming characters are saved until a newline
 character is received.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it.
    // if the string has not been acknowledged, ignore incoming chars
    if (inChar == '\n') {
      stringComplete = true;
      inputString.trim(); // if incoming string uses CR/LF, then this removes the CR
    } else if (stringComplete == false) {
      // add it to the inputString:
      inputString += inChar;
    }
  }
} // end serialEvent


/*
 * blinker
 *  function for making an led blink
 */
void blinker(int LED_pin, int numblink) {
  for (int i=1; i <= numblink; i++) {
    digitalWrite(LED_pin, LOW);
    delay(150);
    digitalWrite(LED_pin, HIGH);
    delay(150);
  }
  digitalWrite(LED_pin, LOW);

  return;
}


/* 
 * applyControl
 *  converts the single PID output to two valve control values
 */
void applyControl(double *PID_ctrl, byte *VA_Out, byte *VE_Out) {

  // Determine output from control signal
  // Use this function to linearize valve action
  // PID_ctrl is output from PID. Has range 0 - 1
  // Valves use PWM output with range 8-bit (0-255)
  // However, valves appear to have poor response to extreme PWM values, so convert
  //  PID_ctrl into two symmetric values between min and max for each valve

  *VA_Out = round(*PID_ctrl * vaSpan) + vaMin;

  *VE_Out = round((1 - *PID_ctrl) * veSpan) + veMin;

  // sends the outputs to the appropiate pins
  analogWrite(VACUUM, *VA_Out);
  analogWrite(VENT, *VE_Out);

}



/*
 * parseWave
 * inserts values into the command array
char wType[max_waveforms] = {'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x'};
int wAmp[max_waveforms] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int wOffset[max_waveforms] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned long wPer[max_waveforms] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float wFreq[max_waveforms] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int wDuty[max_waveforms] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned long wTime[max_waveforms] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
 */
void parseWave(String cmdstr, unsigned long *wPer, double *wFreq, int *wAmp, int *wOffset, int *wDuty, unsigned long *wTime) {

  String arg;
  int beginIdx = 0;
  int idx = 0;

  // Note: toInt() returns a long (? seems to work...)

  idx = cmdstr.indexOf(",",beginIdx);
  arg = cmdstr.substring(beginIdx, idx);
  *wPer = arg.toInt();                // in ms
  *wFreq = 1 / (double)*wPer * 1000;  // in Hz
  beginIdx = idx + 1;
  
  idx = cmdstr.indexOf(",",beginIdx);
  arg = cmdstr.substring(beginIdx, idx);
  *wAmp = arg.toInt();
  beginIdx = idx + 1;

  idx = cmdstr.indexOf(",",beginIdx);
  arg = cmdstr.substring(beginIdx, idx);
  *wOffset = arg.toInt();
  beginIdx = idx + 1;

  idx = cmdstr.indexOf(",",beginIdx);
  arg = cmdstr.substring(beginIdx, idx);
  *wDuty = arg.toInt();
  beginIdx = idx + 1;

  idx = cmdstr.indexOf(",",beginIdx);
  arg = cmdstr.substring(beginIdx, idx);
  *wTime = arg.toInt(); 

  return;
}

/*
 initWaveforms
  initializes the arrays that hold the list of waveforms
char wType[max_waveforms] = {'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x'};
int wAmp[max_waveforms] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int wOffset[max_waveforms] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned long wPer[max_waveforms] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float wFreq[max_waveforms] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int wDuty[max_waveforms] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned long wTime[max_waveforms] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
 */
void initWaveforms(byte max_waveforms, int *index_prog, int *index_cmd, char wType[], int wAmp[], int wOffset[], unsigned long wPer[], double wFreq[], int wDuty[], unsigned long wTime[]) {
  *index_prog = 0;
  *index_cmd = 0;
  for (int idx = 0; idx < max_waveforms; idx++) {
    wType[idx] = 'x';
    wAmp[idx] = 0;
    wOffset[idx] = 0;
    wPer[idx] = 0;
    wFreq[idx] = 0;
    wDuty[idx] = 0;
    wTime[idx] = 0;
  }
}
 
/*
 squareWave
 defines a square wave immediate output value based on wave parameters and time 
 */
int squareWave(unsigned long per, int amp, int offset, int duty, unsigned long time_run) {

  //Serial.print("time_run: "); Serial.println(time_run, DEC);
  //Serial.print("period: "); Serial.println(per, DEC);
  //Serial.print("amplitude: "); Serial.println(amp, DEC);
  //Serial.print("duty: "); Serial.println(duty, DEC);
  float wave_pos = time_run % per;
  //Serial.print("wave_pos: "); Serial.println(wave_pos, DEC);
  float wave_fraction = wave_pos / (float)per * 100; // *100 because duty cycle is int
  //Serial.print("wave_fraction: "); Serial.println(wave_fraction, DEC);
  int output;
  if (duty > wave_fraction) {
    output = amp + offset;
  } else {
    output = offset - amp;
  }
  //Serial.print("output: "); Serial.println(output, DEC);
  return output;
}

/*
 rampWave
 defines a ramp wave (sawtooth) immediate output value based on wave parameters and time 
 */
int rampWave(unsigned long per, int amp, int offset, int duty, unsigned long time_run) {

  //Serial.print("time_run: "); Serial.println(time_run, DEC);
  //Serial.print("period: "); Serial.println(per, DEC);
  //Serial.print("amplitude: "); Serial.println(amp, DEC);
  unsigned long wave_pos = time_run % per;
  //Serial.print("wave_pos: "); Serial.println(wave_pos, DEC);
  float m = (float)amp * 2 / (float)per;
  //Serial.print("m: "); Serial.println(m, DEC);
  int output = m * wave_pos + offset - amp;   // Note: the slope does not change, does not need to be computed every loop
  //Serial.print("output: "); Serial.println(output, DEC);
    
  return output;
}
