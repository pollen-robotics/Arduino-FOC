#include "BLDCDriver3PWM.h"

BLDCDriver3PWM::BLDCDriver3PWM(int phA, int phB, int phC, int en1, int en2, int en3){
  // Pin initialization
  pwmA = phA;
  pwmB = phB;
  pwmC = phC;

  // enable_pin pin
  enableA_pin = en1;
  enableB_pin = en2;
  enableC_pin = en3;

  // default power-supply value
  voltage_power_supply = DEF_POWER_SUPPLY;
  voltage_limit = NOT_SET;
  pwm_frequency = NOT_SET;

}

// enable motor driver
void  BLDCDriver3PWM::enable(){
    // enable_pin the driver - if enable_pin pin available
    if ( _isset(enableA_pin) ) digitalWrite(enableA_pin, enable_active_high);
    if ( _isset(enableB_pin) ) digitalWrite(enableB_pin, enable_active_high);
    if ( _isset(enableC_pin) ) digitalWrite(enableC_pin, enable_active_high);
    // set zero to PWM
    setPwm(0,0,0);
}

// disable motor driver
void BLDCDriver3PWM::disable()
{
  // set zero to PWM
  setPwm(0, 0, 0);
  // disable the driver - if enable_pin pin available
  if ( _isset(enableA_pin) ) digitalWrite(enableA_pin, !enable_active_high);
  if ( _isset(enableB_pin) ) digitalWrite(enableB_pin, !enable_active_high);
  if ( _isset(enableC_pin) ) digitalWrite(enableC_pin, !enable_active_high);

}

// init hardware pins
int BLDCDriver3PWM::init() {
  // PWM pins
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  if( _isset(enableA_pin)) pinMode(enableA_pin, OUTPUT);
  if( _isset(enableB_pin)) pinMode(enableB_pin, OUTPUT);
  if( _isset(enableC_pin)) pinMode(enableC_pin, OUTPUT);


  // sanity check for the voltage limit configuration
  if(!_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

  // Set the pwm frequency to the pins
  // hardware specific function - depending on driver and mcu
  params = _configure3PWM(pwm_frequency, pwmA, pwmB, pwmC);
  initialized = (params!=SIMPLEFOC_DRIVER_INIT_FAILED);
  return params!=SIMPLEFOC_DRIVER_INIT_FAILED;
}

/*

// init hardware pins
int BLDCDriver3PWM::init_not_timer() {
  // PWM pins
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);
  if( _isset(enableA_pin)) pinMode(enableA_pin, OUTPUT);
  if( _isset(enableB_pin)) pinMode(enableB_pin, OUTPUT);
  if( _isset(enableC_pin)) pinMode(enableC_pin, OUTPUT);


  // sanity check for the voltage limit configuration
  if(!_isset(voltage_limit) || voltage_limit > voltage_power_supply) voltage_limit =  voltage_power_supply;

  // Set the pwm frequency to the pins
  // hardware specific function - depending on driver and mcu
  // params = _configure3PWM(pwm_frequency, pwmA, pwmB, pwmC);

  HardwareTimer* HT1 = _initPinPWM(pwm_frequency, pinTimers[0]);
  HardwareTimer* HT2 = _initPinPWM(pwm_frequency, pinTimers[1]);
  HardwareTimer* HT3 = _initPinPWM(pwm_frequency, pinTimers[2]);

  uint32_t channel1 = STM_PIN_CHANNEL(pinTimers[0]->function);
  uint32_t channel2 = STM_PIN_CHANNEL(pinTimers[1]->function);
  uint32_t channel3 = STM_PIN_CHANNEL(pinTimers[2]->function);

  STM32DriverParams* params = new STM32DriverParams {
    .timers = { HT1, HT2, HT3 },
    .channels = { channel1, channel2, channel3 },
    .pwm_frequency = pwm_frequency
  };
  timerPinsUsed[numTimerPinsUsed++] = pinTimers[0];
  timerPinsUsed[numTimerPinsUsed++] = pinTimers[1];
  timerPinsUsed[numTimerPinsUsed++] = pinTimers[2];

  _alignTimersNew();


  initialized = (params!=SIMPLEFOC_DRIVER_INIT_FAILED);
  return params!=SIMPLEFOC_DRIVER_INIT_FAILED;
}
*/


// Set voltage to the pwm pin
void BLDCDriver3PWM::setPhaseState(int sa, int sb, int sc) {
  // disable if needed
  if( _isset(enableA_pin) &&  _isset(enableB_pin)  && _isset(enableC_pin) ){
    digitalWrite(enableA_pin, sa == _HIGH_IMPEDANCE ? LOW : HIGH);
    digitalWrite(enableB_pin, sb == _HIGH_IMPEDANCE ? LOW : HIGH);
    digitalWrite(enableC_pin, sc == _HIGH_IMPEDANCE ? LOW : HIGH);
  }
}

// Set voltage to the pwm pin
void BLDCDriver3PWM::setPwm(float Ua, float Ub, float Uc) {

  // limit the voltage in driver
  Ua = _constrain(Ua, 0.0f, voltage_limit);
  Ub = _constrain(Ub, 0.0f, voltage_limit);
  Uc = _constrain(Uc, 0.0f, voltage_limit);
  // calculate duty cycle
  // limited in [0,1]
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  // hardware specific writing
  // hardware specific function - depending on driver and mcu
  _writeDutyCycle3PWM(dc_a, dc_b, dc_c, params);
}
