/*
 * Developed as part of the UCL ENGF0001 module.
 * Integration of software and Arduino-ESP connectivity code
 * by Ioan Steffan Thomas.
 * Stirring subsystem code by Sameer Kurbanov.
 * Heating subsystem code by Ceto Kim.
 * pH subsystem by code Ibraheem Siddiqui.
 */


/*
 * ESP - Arduino connectivity done with I2C protocol.
 * https://docs.arduino.cc/learn/communication/wire/
 * The data lines (SDA) and clock lines (SCL) should be 
 * connected via a level-shifter between Arduino and ESP.
 * The level-shifter should also connect the ground (GND)
 * pins.
 * Arduino Uno pins:
 *    - SDA: SDA/A4
 *    - SCL: SCL/A5
 * The Arduino is the device given an address of 1.
 */


#include "data.h"
#include <Wire.h>
#include <math.h>

#define MONITOR_BAUD_RATE 115200
#define ARDUINO_ADDRESS 1

// Heating globals, constants, and macros.
#define PWM_PIN 3
#define THERMISTOR_PIN A0

const float R1 = 9940;
const float c1 = 0.001129148, c2 = 0.0002347118632678, c3 = 0.0000000876741;
const float kp = 0.3;
const float ki = 12.0;

float set_temperature = 30.0;
float current_temperature = 0.0;
float PID_error = 0.0;
float PID_i = 0.0;
float PID_value = 0.0;

float error, Vheater;
long prevtime, T1;

unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 200;

unsigned long lastSerialUpdateTime = 0;
const unsigned long serialUpdateInterval = 1500;
// End heating globals, constants, and macros.


// Stirring globals, constants, and macros.
// Located after class definition.
// End stirring globals, constants, and macros.


// pH globals, constants, and macros.
#define pHPin A1
#define acidPumpPin 12
#define basePumpPin 13
#define phS 7 //ph of standard solution
#define Es 1026.393  //Electrical potential at reference or standard electrode
#define F (9.6485309*10000) //Faraday constant
#define R 8.314510 //universal gas constant
#define T_ph 298 //Temperature in Kelvin is assumed to be constant
#define sendPHInterval 100 // Sends the pH every 100 milliseconds
#define pumpCheckInterval 2000 //How often the pump is activate when outside the pH range, to give time for stirring
#define pumpActivationInterval 2500 //The pump is activated for (pumpActivationInterval - pumpCheckInterval) milliseconds

float targetpH = 5; //Initial target pH
float pHHi = 5.20; //NB: Tolerance range
float pHLo = 4.80;
float pH;
bool pumpActivated = false;
// End pH globals, constants, and macros.


Data bioreactor_data;
Data target_data;

void start_serial()
{
  Serial.end();
  Serial.begin(MONITOR_BAUD_RATE);
  // Waits for the serial connection to be secured.
  while (!Serial);
  // Adds initial delay so the serial monitor will receive any messages (change as needed).
  delay(1000);
}

void data_setup()
{
  target_data.ph = String(7);
  target_data.temp = String(25);
  target_data.rpm = String(0);
  bioreactor_data.ph = String(0);
  bioreactor_data.temp = String(0);
  bioreactor_data.rpm = String(0);
  Serial.println("Data setup successful.");
}

void wire_setup()
{
  Wire.begin(ARDUINO_ADDRESS);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  Serial.println("Wire serial I2C setup successful.");
}

void requestEvent()
{
  String message = bioreactor_data.ph + "," + bioreactor_data.temp + "," + bioreactor_data.rpm + "\n";

  // Testing.
  // message = "55,235,6900\n"; // Remove once bioreactor_data is being set.
  // Testing.

  for (int i = 0; i < message.length(); i++){
    Wire.write(message[i]);
  }

}

void receiveEvent(int howMany)
{
  String data = String("");
  int count = 0;
  while (1 < Wire.available())
  {
    char c = Wire.read();
    if (c == ',' || c == '\n')
    {
      if (data == "-1") // does this work in cpp? doesn't in c but I don't know
      {
        break;
      }
      switch (count++)
      {
        case 0:
          target_data.ph = data;
          break;
        case 1:
          target_data.temp = data;
          break;
        case 2:
          target_data.rpm = data;
          break;
        default:
          break;
      }
      data = String("");
    }
    else
    {
      data += c;// does this work in cpp? doesn't in c but I don't know
    }
  }
}

/*
 * get_target<> functions work given that: 
 *    0 <= ph < 10 (with maximum 3 decimal places).
 *   10 <= temp < 100 (with maximum 4 total digits).
 *    0 <= rpm < 10000 (with no decimal places).
 */
 
float get_target_ph()
{
  return target_data.ph.toInt() / pow(10.0, target_data.ph.length() - 1.0);
}

float get_target_temp()
{
  
  return target_data.temp.toInt() / pow(10.0, target_data.ph.length() - 2.0);
}

int get_target_rpm()
{
  return target_data.rpm.toInt();
}

/*
 * set_<> functions work given that: 
 *    0 <= ph < 10 (with maximum 3 decimal places).
 *   10 <= temp < 100 (with maximum 4 total digits).
 *    0 <= rpm < 10000 (with no decimal places).
 */

void set_ph(float ph)
{
  bioreactor_data.ph = String((int)round(ph * 1000));
}

void set_temp(float temp)
{
  bioreactor_data.temp = String((int)round(temp * 100));
}

void set_rpm(int rpm)
{
  bioreactor_data.rpm = String(rpm);
}


// Heating.

void heatingSetup() 
{
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);
  Serial.println("Heating setup successful.");
}

void heatingLoop() 
{
  unsigned long currtime = micros();
  float deltaT = (currtime - prevtime) * 1e-6;
  if (Serial.available() > 0) {
    float newTemperature = get_target_temp();
    if (newTemperature >= 25 && newTemperature <= 35) {
      set_temperature = newTemperature;
      // Serial.print("New set temperature updated to: ");
      // Serial.print(set_temperature);
      // Serial.println(" 째C");
    } 
    // else {
    //   Serial.println("Invalid temperature. Must be value between 25째C and 35째C.");
    // }
  }

  if (currtime - T1 > 0) {
    prevtime = currtime;
    T1 = T1 + 10000; // 10 ms update period
    current_temperature = readThermistor();
    set_temp(current_temperature);
    PID_error = set_temperature - current_temperature;
    Vheater = round(204 * (kp * PID_error)); // PI controller: scale by 1023 levels/5 V DAC resolution
    Vheater = constrain(Vheater, 0, 1023); // Limit motor voltage to 5 V
    if (Vheater == 255) { Vheater = 256; } // Work-around for bug in Nano/Uno with 10-bit configuration
    analogWrite(PWM_PIN, Vheater);
  } 

  if (currtime - lastSerialUpdateTime >= serialUpdateInterval) {
    lastSerialUpdateTime = currtime;
    // Serial.print("Set Temp: ");
    // Serial.print(set_temperature);
    // Serial.print(" C, Current Temp: ");
    // Serial.print(current_temperature);
    // Serial.print(" C, PID Output: ");
    // Serial.println(Vheater);
  }
}

float readThermistor() 
{
  int Vo = analogRead(THERMISTOR_PIN);
  float R2 = R1 * (1023.0 / (float)Vo - 1.0);
  float logR2 = log(R2);
  float T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  T = T - 274.1;
  return T;
}

// End Heating


// Stirring.

class MotorController 
{
private:
    // Motor and sensor constants
    const float RPM_PER_VOLT = 225;  
    const float MOTOR_RESPONSE_TIME = 0.15;
    const float PULSES_PER_REV = 70;        // Number of hall sensor pulses per revolution (70 pulses means 140 total rise and fall edges)
    const int MAX_RPM = 1400;               // The motor used for this project could only handle up to around 1300 RPM
    const int MIN_PULSE_TIME = 6e7 / MAX_RPM / PULSES_PER_REV;
    const float FREQ_TO_RPM = 60 / PULSES_PER_REV;

    // PI Controller parameters
    const float DAMPING_RATIO = 1.0;
    const float NATURAL_FREQ;
    const float PROP_GAIN;
    const float INT_GAIN;

    // Pin configurations
    const byte ENCODER_PIN;
    const byte MOTOR_PIN;

    // RPM variables
    float targetRPM = 0;
    float measuredRPM = 0;
    float averageRPM = 0;
    float frequency = 0;
    float integralError = 0;
    
    // Timing variables
    long currentTime = 0;
    long previousTime = 0;
    long pulseTime = 0;
    long previousPulseTime = 0;
    long nextUpdateTime = 0;

public:
    MotorController(byte encoderPin, byte motorPin) : 
        ENCODER_PIN(encoderPin),
        MOTOR_PIN(motorPin),
        NATURAL_FREQ(1 / MOTOR_RESPONSE_TIME),
        /* This stirring mechanism uses a PI control system (proportional-integral), and as such the equations for these can be found online.
        PROP_GAIN and INT_GAIN represent the proportional and integral terms, often referred to as Kp and Ki in mathematical notation.*/
        PROP_GAIN((2 * DAMPING_RATIO * NATURAL_FREQ / (NATURAL_FREQ) - 1) / RPM_PER_VOLT),
        INT_GAIN(NATURAL_FREQ * NATURAL_FREQ / RPM_PER_VOLT / (1 / MOTOR_RESPONSE_TIME))
    {
        initialise();
    }

    void initialise() 
    {
        pinMode(ENCODER_PIN, INPUT_PULLUP);
        pinMode(MOTOR_PIN, OUTPUT);
        
        TCCR1A = 0b00000011;
        TCCR1B = 0b00000001;
        
        analogWrite(MOTOR_PIN, 0);
    }

    void handleSerialInput() 
    {
        if (Serial.available() > 0)
         {
            String input = Serial.readStringUntil('\n');
            input.trim();
            float newSpeed = input.toFloat();
            
            if (newSpeed >= 0 && newSpeed <= MAX_RPM) 
            {
                targetRPM = newSpeed;
            } 
         }
    }

    void updateMotorControl() 
    {
        targetRPM = get_target_rpm();
        currentTime = micros();
        // The "1000" represents 1 milisecond hence the update frequency here is 1000 times per second or 1000hz.
        if (currentTime - nextUpdateTime > 1000) 
        {
            float diffTime = (currentTime - previousTime) * 1e-6;
            previousTime = currentTime;
            nextUpdateTime += 1000;  

            updateSpeed(diffTime);
            applyMotorControl();
            // outputDebugInfo();
        }
    }

    void pulseDetected() 
    {
        pulseTime = micros();
        if (abs(pulseTime - previousPulseTime) > MIN_PULSE_TIME)
        {
            frequency = 0.75 * frequency + 2.5e5 / float(pulseTime - previousPulseTime);
            previousPulseTime = pulseTime;
        }
    }

private:
    void updateSpeed(float diffTime) 
    {
        measuredRPM = frequency * FREQ_TO_RPM;
        
        if (currentTime - pulseTime > 1e5) 
        {
            measuredRPM = 0;
            frequency = 0;
        }
        
        set_rpm((int)measuredRPM);

        float error = targetRPM - measuredRPM;
        integralError = constrain(integralError + INT_GAIN * error * diffTime, 0, 5);
        averageRPM = 0.1 * measuredRPM + 0.9 * averageRPM;
    }

    void applyMotorControl() 
    {
        float error = targetRPM - measuredRPM;
        int motorVoltage = round(204 * (PROP_GAIN * error + integralError));
        motorVoltage = constrain(motorVoltage, 0, 1023);
        
        // This is a supposed work-around for Arduino Uno 10-bit PWM bug
        if (motorVoltage == 255) motorVoltage = 256;
        
        analogWrite(MOTOR_PIN, motorVoltage);
    }

    void outputDebugInfo() 
    {
        Serial.print("Target RPM: ");
        Serial.print(targetRPM);
        Serial.print("| ");
        Serial.print("Average RPM: ");
        Serial.print(averageRPM);
        Serial.print("| ");
        Serial.print("Measured RPM: ");
        Serial.print(measuredRPM);
        Serial.print("| ");
        Serial.print("Error: ");
        Serial.println(abs(targetRPM - averageRPM));
    }
};

MotorController *controller;

void pulseInterrupt() 
{
    controller->pulseDetected();
}

void stirringSetup()
{
  controller = new MotorController(2, 10);
  attachInterrupt(digitalPinToInterrupt(2), pulseInterrupt, RISING);
  Serial.println("Stirring setup successful.");
}

void stirringLoop()
{
  // controller->handleSerialInput();
  controller->updateMotorControl();
}

// End stirring.


// pH.

void pHSetup()
{
  // Serial.println("pH Connected");
  pinMode(acidPumpPin,OUTPUT);
  pinMode(basePumpPin,OUTPUT);
  turnOffPumps();
  // Serial.println("5 second delay to activate power")
  delay(5000);
  Serial.println("pH setup successful.");
}


void pHLoop()
{
  static unsigned long sendPHTime = millis();
  static unsigned long checkPumpTime = millis();
  static unsigned long ActivatedPumpTime = millis();
  pH = pHMain();
  if (1) 
  { //Gets the new target pH
    targetpH = get_target_ph();
    pHHi = targetpH + 0.2;
    pHLo = targetpH - 0.2;
  }
  if (millis() - sendPHTime > sendPHInterval) 
  {
    set_ph(pH);
    // Serial.print("data sent");
    sendPHTime = millis();
  }
  
  if (!pumpActivated)
  {
    if (millis() - checkPumpTime > pumpCheckInterval)
    {
      pumpActivated = activatePump(pH,pHHi,pHLo);
      checkPumpTime = millis();
    }
  }
  else 
  {
    if (millis() - ActivatedPumpTime > pumpActivationInterval) 
    {
      turnOffPumps();
      pumpActivated = false;
      ActivatedPumpTime = millis();
      checkPumpTime = millis();
    }
  }
}

//Handles the pH calculation
float pHMain() {
  int phv=analogRead(pHPin);
  float Ex = phv*5000.0/1023;
  float ln = log(10); //Arduino log(x) refers to the natural logarithm
  float phX = phS + ((Es-Ex)*F)/(R*2000*T_ph*ln); //Probe sensitivity is of by the factor of 2000
  // Serial.print("current pH value:     ");
  // Serial.println(phX);
  return phX;
}

bool activatePump(float pH, float pHHi, float pHlo) //activates in pulses, to give time for pH to normalise after some stirring
{
  if (pH > pHHi) 
  { 
    digitalWrite(acidPumpPin, HIGH);
    // Serial.println("Acid pump activated");
    return true;
  } 
  if (pH < pHLo) 
  {
    digitalWrite(basePumpPin, HIGH);
    // Serial.println("Base pump activated");
    return true;
  }
  return false; 
}

void turnOffPumps() 
{
  digitalWrite(acidPumpPin,LOW);
  digitalWrite(basePumpPin,LOW);
  // Serial.println("Pumps deactivated.");
}

// End pH.

void setup() 
{
  start_serial();

  wire_setup();

  heatingSetup();
  stirringSetup();
  pHSetup();

  TCCR1A = 0b00000011; // 10-bit
  TCCR1B = 0b00000001; // 7.8 kHz

  Serial.println("Setup successful.");
  delay(3000);
}

void loop() 
{
  heatingLoop();
  stirringLoop();
  pHLoop();

  // Testing.
  Serial.println("TP:" + String(get_target_ph()));
  Serial.println("TT:" + String(get_target_temp()));
  Serial.println("TR:" + String(get_target_rpm()));

  Serial.println("BP:" + bioreactor_data.ph);
  Serial.println("BT:" + bioreactor_data.temp);
  Serial.println("BR:" + bioreactor_data.rpm);
  // End testing.
}
