#include "main.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define BITMASK(nbits) ((1 << nbits) - 1)

// basic pins
#define INPUT  00
#define OUTPUT 01
#define MEDIUM 01
#define HIGH 1
#define LOW  0
// PORT A used pins
#define PA0   0
#define PA1   1
#define PA2   2
#define PA3   3
#define PA4   4
#define PA5   5
#define PA6   6
#define PA7   7
#define PA12  8
#define PA15  9
// PORT B used pins
#define PB0  10
#define PB1  11
#define PB4  12
#define PB5  13
// PORT ID
#define PORT_A 0
#define PORT_B 1

// PORT / Pin ID, Pin number
uint8_t pinInfoArr[][3] = {
  // PORTA
  {PORT_A, 0, 6},    // PA0
  {PORT_A, 1, 7},    // PA1
  {PORT_A, 2, 8},    // PA2
  {PORT_A, 3, 9},    // PA3
  {PORT_A, 4, 10},   // PA4
  {PORT_A, 5, 11},   // PA5
  {PORT_A, 6, 12},   // PA6
  {PORT_A, 7, 13},   // PA7
  {PORT_A, 12, 22},  // PA12
  {PORT_A, 15, 25},  // PA15
  // PORTB
  {PORT_B, 0, 14},   // PB0
  {PORT_B, 1, 15},   // PB1
  {PORT_B, 4, 27},   // PB4
  {PORT_B, 5, 28},   // PB5
};

// state indicator
#define RED_LED     PA15
#define YELLOW_LED  PB4
#define GREEN_LED   PB5
#define STATIC 1
#define BLINK  2
#define CLEAR  3
#define defBlinkTime 800

// Motion drive
#define mStepPin         PB1
#define mDirectionPin    PB0
#define mDriveStartPin   PA12
#define mDefaultHTime    80
#define mDefaultSpeed    140 // 140 step per x1 sec
#define mLowSpeed        60  // 60 step per x1 sec (for calibrate mode)
#define START            1
#define STOP             0
#define UP               1
#define DOWN             0
#define ACCELERATE       1
#define DEACCELERATE     2

// Floor sensor
#define F1_Sensor_Pin   GPIO_PIN_3
#define F2_Sensor_Pin   GPIO_PIN_4
#define F3_Sensor_Pin   GPIO_PIN_5
#define F0_Sensor_Pin   GPIO_PIN_6
#define F_Sensor_Trigg  0

// Btn
#define F1_Btn_Pin      GPIO_PIN_0
#define F2_Btn_Pin      GPIO_PIN_1
#define F3_Btn_Pin      GPIO_PIN_2

// Floor brd
uint8_t FLOOR_I2C_ADDR[] = {0x22, 0x21, 0x20}; // F1-F3
// PCA9534 registers
#define PCA_CONFIGR 0x03
#define PCA_OUTPUTR 0x01
// Floor ID
#define FLOOR_1           1
#define FLOOR_2           2
#define FLOOR_3           3
#define FLOOR_ALL         4
// Frames
#define FRAME_1           1
#define FRAME_2           2
#define FRAME_3           3
#define FRAME_DP          4
#define FRAME_LAMP        5
#define FRAME_CLEAR       6
#define FRAME_LAMP_CLEAR  7
#define LAMP_BIT_MASK     0x2


I2C_HandleTypeDef hi2c1;
unsigned long msTicks = 0;

// Led indicator
uint8_t ledIndicator_VAR_mode = 0;
uint8_t ledIndicator_VAR_ledName = 0;
uint16_t ledIndicator_VAR_blinkT = 0;
// Motion drive
uint8_t mDrive_VAR_curDir      = 0;
uint8_t mDrive_VAR_DriveActive = 0;
uint16_t mDrive_VAR_mHighTime  = 0;
uint16_t mDrive_VAR_mLowTime   = 0;
uint8_t  accelerate_Mode = 0;
uint16_t accelerate_incrementStepInterval, accelerate_stepTime, accelerate_nomSpeed;
// Floor sensor
uint8_t floorSensor_VAR_StateArr[4];
uint8_t floorSensor_VAR_CurrentFloor = 0;
uint8_t floorSensor_VAR_activeCalibrate = 0;
// Btn
uint8_t btnControl_VAR_StateArr[4]; // #0 index has not used (1 - F1 / 2 - F2 / 3 - F3)
uint8_t btnControl_VAR_pressAccess = 1;
// Trip control
uint8_t tripControl_VAR_activeTrip = 0;
uint8_t tripControl_VAR_driveLatch = 0;
uint8_t tripControl_VAR_floorCount = 0;
uint8_t tripControl_VAR_assignedFloor = 0;


// HAL function
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

// User system function
static inline uint32_t millis();
static inline uint32_t micros();
void pinControl_setMode(unsigned pin, unsigned mode);
void pinControl_setSpeed(unsigned pin, unsigned speed);
void pinControl_setState(unsigned pin, unsigned mode);
uint8_t pinControl_getState(unsigned pin);
void ledIndicator_initPins(void);
void ledIndicator_setMode(uint8_t mode, uint8_t ledName, uint16_t blinkTime);
void ledIndicator_loopBlink(void);
void motionDrive_Init(void);
void motionDrive_Loop(void);
uint8_t motionDrive_Control(uint8_t mode, uint8_t direction);
uint8_t motionDrive_setSpeed(uint16_t mHighTime, uint16_t mLowTime);
uint16_t motionDrive_speedToLowTime(uint16_t summSteps, uint16_t hTime);
uint16_t motionDrive_getAccelerateSpeed(uint16_t nomSpeed, int8_t changePercent);
void motionDrive_accelerateControl(uint8_t mode, uint16_t nomSpeed, uint8_t changePercent, uint8_t accelerateTime);
void motionDrive_accelerateLoop();
void floorSensor_Init();
void floorSensor_Read();
uint8_t floorSensor_NeedCalibrate();
void floorSensor_calibrateControl(uint8_t direction);
void floorSensor_calibrateLoop();
void btnControl_Init();
void btnControl_Read();
void floorBoard_Init();
uint8_t floorBoard_BitSave(uint8_t lastFrame, uint8_t newFrame, uint8_t mode);
void floorBoard_Control(uint8_t floor, uint8_t frame);
uint8_t tripControl_checkRunPossiple(uint8_t pressedFloor);
void tripControl_start(uint8_t assignedFloor);
void tripControl_complete();
void tripControl_supervisor();
void updCurrentFloor();
// Events
void btnControl_PressEvent(uint8_t pressedFloor);


int main(void) {

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();

  RCC->AHBENR |= (1 << 17) | (1 << 18); // PORTA and PORTB clock enabled

  // Config 1ms tick timer
  #define coreFrequency 24000000 // 24mHz core frequency
  SysTick_Config(coreFrequency / 1000);

  HAL_Delay(2000);

  // Init all modules
  floorBoard_Init();
  ledIndicator_initPins();
  floorSensor_Init();
  motionDrive_Init();
  btnControl_Init();

  // ON yellow led
  ledIndicator_setMode(STATIC, YELLOW_LED, 0);
  HAL_Delay(2500);

  // Get current floor
  updCurrentFloor();

  // Need calibration?
  uint8_t needCalibrateState = floorSensor_NeedCalibrate();
  switch (needCalibrateState) {
    case 0: // not need calibration
      ledIndicator_setMode(STATIC, GREEN_LED, 0);
     break;
    case 1: // down calibration
      floorSensor_calibrateControl(DOWN);
     break;
    case 2: // down calibration (risk)
      floorSensor_calibrateControl(DOWN);
     break;
    case 3: // up calibration
      floorSensor_calibrateControl(UP);
     break;
    case 4: // fault all sensor. Not operation?
      tripControl_VAR_driveLatch = 1;
      btnControl_VAR_pressAccess = 0;
      ledIndicator_setMode(BLINK, RED_LED, 550);
     break;
  }

  HAL_Delay(5000);

  // Loop
  while (1) {
    ledIndicator_loopBlink();
    floorSensor_Read();
    updCurrentFloor();
    btnControl_Read();
    motionDrive_Loop();
    motionDrive_accelerateLoop();
    tripControl_supervisor();
    floorSensor_calibrateLoop();
  }

}


// Functions section

// User functions
// ### TIMERS ###
// # User millis() timer
// Return current ms ticks value (ulong type)
static inline uint32_t millis() {
  return msTicks;
}
// # User micros() timer
// Return current micros ticks value (ulong type)
static inline uint32_t micros() {
  uint32_t ms;
  uint32_t st;
  do {
    ms = msTicks;
    st = SysTick->VAL;
    asm volatile("nop");
    asm volatile("nop");
  } while (ms != msTicks);
  return ms * 1000 - st / ((SysTick->LOAD + 1) / 1000);
}

// ### PIN control ###
// # Set pin mode
// Ex: pinControl_SetMode(28, OUTPUT); - set PB5 to OUTPUT mode
void pinControl_setMode(unsigned pin, unsigned mode) {
  uint8_t pinArrPort = pinInfoArr[pin][0];
  uint8_t pinArrID = pinInfoArr[pin][1];

  // get port
  if(pinArrPort == PORT_A) {
    GPIOA->MODER &= ~(BITMASK(2) << (2 * pinArrID));  // Clear bit
    GPIOA->MODER |= ((mode & BITMASK(2)) << (2 * pinArrID)); // Set bit
  } else if(pinArrPort == PORT_B) {
    GPIOB->MODER &= ~(BITMASK(2) << (2 * pinArrID));  // Clear bit
    GPIOB->MODER |= ((mode & BITMASK(2)) << (2 * pinArrID)); // Set bit
  }
}
// # Set pin speed
// Ex: pinControl_SetSpeed(GPIOB, 5, 01); - set PB5 to Medium speed
void pinControl_setSpeed(unsigned pin, unsigned speed) {
  uint8_t pinArrPort = pinInfoArr[pin][0];
  uint8_t pinArrID = pinInfoArr[pin][1];

  // get port
  if(pinArrPort == PORT_A) {
    GPIOA->OSPEEDR &= ~(BITMASK(2) << (2 * pinArrID));
    GPIOA->OSPEEDR |= ((speed & BITMASK(2)) << (2 * pinArrID));
  } else if(pinArrPort == PORT_B) {
    GPIOB->OSPEEDR &= ~(BITMASK(2) << (2 * pinArrID));
    GPIOB->OSPEEDR |= ((speed & BITMASK(2)) << (2 * pinArrID));
  }
}
// # Set pin state
// Ex: pinControl_SetState(28, HIGH); - set PB5 to HIGH state
void pinControl_setState(unsigned pin, unsigned mode) {
  uint8_t pinArrPort = pinInfoArr[pin][0];
  uint8_t pinArrID = pinInfoArr[pin][1];

  switch (mode) {
    case HIGH:
       if(pinArrPort == PORT_A) { GPIOA->BSRR |= (BITMASK(1) << (1 * pinArrID)); }
       else if(pinArrPort == PORT_B) { GPIOB->BSRR |= (BITMASK(1) << (1 * pinArrID)); }
      break;
    case LOW:
       if(pinArrPort == PORT_A) { GPIOA->BSRR |= (BITMASK(1) << (1 * 16 + pinArrID)); }
       else if(pinArrPort == PORT_B) { GPIOB->BSRR |= (BITMASK(1) << (1 * 16 + pinArrID)); }
      break;
  }
}
// # Get pin state
// Ex: pinControl_getState(PB5); - get PB5 state
/*uint8_t pinControl_getState(unsigned pin) {
  uint8_t pinArrPort = pinInfoArr[pin][0];
  uint8_t pinArrNumber = pinInfoArr[pin][2];
  uint8_t pinState = HAL_GPIO_ReadPin(pinArrPort, pinArrNumber);
  return pinState;
}*/

// ### State indicator ###
// # Init indication LED's
// Doc: this function has set LED OUTPUT mode and LOW state
void ledIndicator_initPins(void) {
  uint8_t ledArray[3] = {RED_LED, YELLOW_LED, GREEN_LED};
  for(uint8_t pinN = 0; pinN < 3; pinN++) {
    pinControl_setMode(ledArray[pinN], OUTPUT);
    pinControl_setSpeed(ledArray[pinN], MEDIUM);
    pinControl_setState(ledArray[pinN], LOW);
  }
}
// # Control indication mode
// Doc: this function selected LED indication mode, set-clear pixels
void ledIndicator_setMode(uint8_t mode, uint8_t ledName, uint16_t blinkTime) {
  uint8_t ledArray[3] = {RED_LED, YELLOW_LED, GREEN_LED};

  // clear all leds and current mode
  for(uint8_t pinN = 0; pinN < 3; pinN++) { pinControl_setState(ledArray[pinN], LOW); }
  ledIndicator_VAR_mode = 0;
  if(mode == CLEAR) { return; } // We have already cleared all LEDs

  // select mode
  switch (mode) {
    case STATIC:
       pinControl_setState(ledName, HIGH);
       ledIndicator_VAR_mode = 1;
      break;
    case BLINK:
       ledIndicator_VAR_ledName = ledName;
       ledIndicator_VAR_blinkT = blinkTime;
       ledIndicator_VAR_mode = 2;
      break;
  }
}
// # LED indication blink mode loop
// Doc: this function selected LED indication mode, set-clear led
void ledIndicator_loopBlink(void) {
  static uint32_t blinkTimer;
  static uint8_t  modeActive;
  uint8_t currentMode = ledIndicator_VAR_mode;


  // start-stop
  if(currentMode == 2 && !modeActive) {
    blinkTimer = millis();
    modeActive = 1;
  }
  if(modeActive && currentMode != 2) {
    blinkTimer = 0;
    modeActive = 0;
    return;
  }

  // loop (modeActive used for alternate function (change led state))
  if(modeActive) {
    if(millis() - blinkTimer >= ledIndicator_VAR_blinkT) {
      if(modeActive == 1) {
        pinControl_setState(ledIndicator_VAR_ledName, HIGH);
        modeActive = 2;
      } else {
        pinControl_setState(ledIndicator_VAR_ledName, LOW);
        modeActive = 1;
      }
      blinkTimer = millis();
    }
  }

}

// ### Motion drive ###
// # Init driver
// Doc: this function init motor driver pins, return 1 (success init)
void motionDrive_Init(void) {
  uint8_t pinArray[3] = {mStepPin, mDirectionPin, mDriveStartPin};
  for(uint8_t pinN = 0; pinN < 3; pinN++) {
    pinControl_setMode(pinArray[pinN], OUTPUT);
    pinControl_setSpeed(pinArray[pinN], MEDIUM);
    pinControl_setState(pinArray[pinN], LOW);
  }
}
// # Loop motion drive
// Doc: Drive step motor. For control - set/reset motionStruct.driveActive flag (1 - EN / 0 - STOP)
// Doc: For change speed - call external change-speed function
void motionDrive_Loop(void) {
  static uint8_t activeProcess;
  static uint8_t stageNumber;
  static uint32_t usTimer;

  #define MotorStepPin_Local 1 // for local used

  // start-stop drive
  if(mDrive_VAR_DriveActive && !activeProcess) {
    pinControl_setState(mDirectionPin, mDrive_VAR_curDir);
    activeProcess = 1;
  }
  if(activeProcess && !mDrive_VAR_DriveActive) {
    pinControl_setState(mStepPin, LOW);
    pinControl_setState(mDirectionPin, LOW);
    usTimer       = 0;
    activeProcess = 0;
    stageNumber   = 0;
    return;
  }

  // drive
  if(activeProcess) {

    // stage #1 - HIGH
    if((stageNumber == 1) && (micros() - usTimer >= mDrive_VAR_mLowTime)) {
      stageNumber = 0;
      // set HIGH state to step pin (PB1)
      GPIOB->BSRR |= (BITMASK(1) << (1 * MotorStepPin_Local));
      usTimer = micros();
    }
    // stage #2 - LOW
    if((stageNumber == 0) && (micros() - usTimer >= mDrive_VAR_mHighTime)) {
      stageNumber = 1;
      // set LOW state to step pin (PB1)
      GPIOB->BSRR |= (BITMASK(1) << (1 * 16 + MotorStepPin_Local));
      usTimer = micros();
    }

  }
}
// # Motion drive set speed
// Doc: This function set speed drive (stepHighTime duration, stepLowTime duration)
// Return: 1 - success, 0 - fault
uint8_t motionDrive_setSpeed(uint16_t mHighTime, uint16_t mLowTime) {
  if(!mHighTime || !mLowTime) { return 0; }
  mDrive_VAR_mHighTime = mHighTime;
  mDrive_VAR_mLowTime  = mLowTime;
  return 1;
}
// # Motion control mode
// Doc: This function control motion driver.
// Ex. call: motionFunc.control(START, UP) - start UP drive + smooth accelerate or motionFunc.control(STOP, DOWN) - stop drive
// Return: 1 - success
uint8_t motionDrive_Control(uint8_t mode, uint8_t direction) {
  mDrive_VAR_curDir = direction;
  switch (mode) {
    case START:
       //motionDrive_setSpeed(5000, 5000);
       pinControl_setState(mDriveStartPin, HIGH);
       mDrive_VAR_DriveActive = START;
      break;
    case STOP:
       motionDrive_setSpeed(5000, 5000); // 5k - random value (not set zero!)
       pinControl_setState(mDriveStartPin, LOW);
       mDrive_VAR_DriveActive = STOP;
      break;
  }
  return 1;
}

// # Convert speed (steps/sec) to low time per x1 step
uint16_t motionDrive_speedToLowTime(uint16_t summSteps, uint16_t hTime) {
  uint16_t summHighTime = hTime * summSteps;
  return ceil((1000000 - summHighTime) / summSteps);
}

// # Get accelerate speed.
// This function has will return the nominal speed value modified by changePercent percent.
// changePercent = 0 (not change) / > 0 (increment) / < 0 (decrement)
uint16_t motionDrive_getAccelerateSpeed(uint16_t nomSpeed, int8_t changePercent) {
  if(changePercent > 0) { return ceil(nomSpeed + ((nomSpeed / 100) * changePercent)); }
  else if(changePercent < 0) { return ceil(nomSpeed - ((nomSpeed / 100) * abs(changePercent))); }
  else { return nomSpeed; }
}

// # Accelerate control
// This function has prepare accelerate mode
// Modes: ACCELERATE - smooth accelerate / DEACCELERATE - smooth deaccelerate / CLEAR - clear accelerate flags
void motionDrive_accelerateControl(uint8_t mode, uint16_t nomSpeed, uint8_t changePercent, uint8_t accelerateTime) {
	uint16_t accelerateSpeed, stepsInAcceleration, incrementStepPerSec;
  switch (mode) {
    case ACCELERATE:
       if(accelerate_Mode == 1) { return; }
       accelerate_nomSpeed = nomSpeed;
       accelerateSpeed = motionDrive_getAccelerateSpeed(nomSpeed, ~changePercent);
       stepsInAcceleration = nomSpeed - accelerateSpeed;
       incrementStepPerSec = stepsInAcceleration / accelerateTime;
       accelerate_incrementStepInterval = ceil(1000000 / incrementStepPerSec);
       accelerate_stepTime = motionDrive_speedToLowTime(stepsInAcceleration, mDefaultHTime) / stepsInAcceleration;
       motionDrive_setSpeed(mDefaultHTime, motionDrive_speedToLowTime(accelerateSpeed, mDefaultHTime));
       accelerate_Mode = 1;
     break;
    case DEACCELERATE:
       if(accelerate_Mode == 2) { return; }
       accelerate_nomSpeed = nomSpeed;
       accelerateSpeed = motionDrive_getAccelerateSpeed(nomSpeed, changePercent);
       stepsInAcceleration = nomSpeed - accelerateSpeed;
       incrementStepPerSec = stepsInAcceleration / accelerateTime;
       accelerate_incrementStepInterval = ceil(1000000 / incrementStepPerSec);
       accelerate_stepTime = motionDrive_speedToLowTime(stepsInAcceleration, mDefaultHTime) / stepsInAcceleration;
       motionDrive_setSpeed(mDefaultHTime, motionDrive_speedToLowTime(accelerateSpeed, mDefaultHTime));
       accelerate_Mode = 2;
     break;
    case CLEAR:
       accelerate_incrementStepInterval = 0;
       accelerate_Mode = 0;
       accelerate_stepTime = 0;
     break;
  }
}

// # Accelerate loop
// This function has control accelerate mode (change speed, close loop)
void motionDrive_accelerateLoop() {
  static uint32_t accelerateTimer;
  static uint16_t nominalSpeed;
  static uint8_t accelerateStart;

  // start timer
  if(accelerate_Mode && !accelerateStart) {
    nominalSpeed = motionDrive_speedToLowTime(accelerate_nomSpeed, mDefaultHTime);
    accelerateTimer = micros();
    accelerateStart = 1;
  }

  if(accelerateStart) {
    // accelerate
    if(accelerate_Mode == 1) {
      if(mDrive_VAR_mLowTime > nominalSpeed) {
        if(micros() - accelerateTimer >= accelerate_incrementStepInterval) {
          mDrive_VAR_mLowTime = mDrive_VAR_mLowTime - accelerate_stepTime;
          accelerateTimer = micros();
        }
      } else {
        accelerateTimer = 0;
        accelerateStart = 0;
        motionDrive_accelerateControl(CLEAR, 0, 0, 0);
      }
    }
    // deaccelerate
    else if(accelerate_Mode == 2) {
      if(mDrive_VAR_mLowTime < nominalSpeed) {
        if(micros() - accelerateTimer >= accelerate_incrementStepInterval) {
          mDrive_VAR_mLowTime = mDrive_VAR_mLowTime + accelerate_stepTime;
          accelerateTimer = micros();
        }
      } else {
        accelerateTimer = 0;
        accelerateStart = 0;
        motionDrive_accelerateControl(CLEAR, 0, 0, 0);
      }
    }
  }

}


// ### Floor sensor ###
// # Floor sensor init
// Docs: this function has configuration sense pin (INPUT), write current state to global array (floorSensor_VAR_StateArr[4])
// Not return
void floorSensor_Init() {
  uint8_t pinArray[4] = {F0_Sensor_Pin, F1_Sensor_Pin, F2_Sensor_Pin, F3_Sensor_Pin};
  // Note: sensors pins has been initias in MX_GPIO_Init()
  for(uint8_t pinN = 0; pinN < 4; pinN++) {
    floorSensor_VAR_StateArr[pinN] = HAL_GPIO_ReadPin(GPIOA, pinArray[pinN]);
  }
}
// # Need calibration?
// Docs: this function has detect to need cabine calibration.
// Return value: 0 - not need calibration / 1 - need down calibration / 2 - fault sensor (fault mode or risk down calibration)
// 3 - need UP calibration
uint8_t floorSensor_NeedCalibrate() {
  uint8_t totalDarkenedSensors = 0;

  // calc total darkened sensors value
  for(uint8_t sensorN = 0; sensorN < 4; sensorN++) {
    if(floorSensor_VAR_StateArr[sensorN] == F_Sensor_Trigg) { totalDarkenedSensors++; }
  }

  // select current event
  // #1 - All sensors has free. Need calibrate (DOWN)
  if(!totalDarkenedSensors) { return 1; }
  // #2 - Dual sensors has triggered. Fault sensors. Risk down calibrate?
  else if(totalDarkenedSensors == 2) { return 2; }
  // #2.1 - All sensors has triggered. Fault.
  else if(totalDarkenedSensors > 2) { return 4; }
  // #3 - F0 has triggered and other sensors has free. Need calibration (UP)
  else if(floorSensor_VAR_StateArr[0] == F_Sensor_Trigg && totalDarkenedSensors < 2) { return 3; }
  // #4 - x1 sensor has triggered (F1 / F2 / F3). Write current floor. Not calibration
  else {
    for(uint8_t sensorN = 0; sensorN < 4; sensorN++) {
      if(floorSensor_VAR_StateArr[sensorN] == F_Sensor_Trigg) { floorSensor_VAR_CurrentFloor = sensorN; }
    }
    return 0;
  }
}
// # Read floor sensor
// Docs: this function has read all floor sensors state. Write to floorSensor_VAR_StateArr[] global array
// To protect against false alarms, a clock pass has been introduced (we register a change in state, skip clock cycles, make sure it is steady, zero it in the array)
void floorSensor_Read() {
  static uint8_t pinArray[4] = {F0_Sensor_Pin, F1_Sensor_Pin, F2_Sensor_Pin, F3_Sensor_Pin};
  static uint8_t sensorChangeState[][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; // {changeRegistred, changeState, tactCounter}
  #define SKIP_TACTS 150 // max 255 (uint8_t)

  for(uint8_t pinN = 0; pinN < 4; pinN++) {
    uint8_t pinState = HAL_GPIO_ReadPin(GPIOA, pinArray[pinN]);
    if((pinState != sensorChangeState[pinN][1]) && (!sensorChangeState[pinN][0])) {
      sensorChangeState[pinN][1] = pinState;
      sensorChangeState[pinN][0] = 1;
    }
    if(sensorChangeState[pinN][0]) {
      if(sensorChangeState[pinN][2] <= SKIP_TACTS) {
        sensorChangeState[pinN][2]++;
      } else {
        if(pinState == sensorChangeState[pinN][1]) {
          floorSensor_VAR_StateArr[pinN] = pinState;
        }
        sensorChangeState[pinN][0] = 0;
        sensorChangeState[pinN][2] = 0;
      }
    }
  }
}
// # Floor sensor calibrate control
// This function has start low drive for search current floor
void floorSensor_calibrateControl(uint8_t direction) {
  ledIndicator_setMode(BLINK, YELLOW_LED, 550);
  //uint16_t lowTime = motionDrive_speedToLowTime(mLowSpeed, mDefaultHTime);
  // 60steps per x1 second
  // 1s = 1000000us
  // 1000000us / 100 = 10000us per x1 step
  // x1 step = LOW time + HIGH time = 16667 / 2 = 8333us (833us - LOW + 833us - HIGH)
  motionDrive_setSpeed(8333, 8333);
  //motionDrive_setSpeed(mDefaultHTime, lowTime);
  motionDrive_Control(START, direction);
  floorSensor_VAR_activeCalibrate = 1;
}
// # Floor sensor calibrate control loop
// This function has checked update current floor per calibrate mode.
// If current floor has exist - stop calibration
void floorSensor_calibrateLoop() {
  if(floorSensor_VAR_activeCalibrate && (floorSensor_VAR_CurrentFloor || !floorSensor_VAR_StateArr[0])) {
    motionDrive_Control(STOP, 0);
    ledIndicator_setMode(STATIC, GREEN_LED, 0);
    floorSensor_VAR_activeCalibrate = 0;
  }
}


// ### Btn ###
// # Floor BTN init
// Docs: this function has configuration btn pin (INPUT), write current state to global array (btnControl_VAR_StateArr[4])
// Not return
void btnControl_Init() {
  uint8_t pinArray[3] = {F1_Btn_Pin, F2_Btn_Pin, F3_Btn_Pin};
  // Note: sensors pins has been initias in MX_GPIO_Init()
  for(uint8_t pinN = 0; pinN < 3; pinN++) {
    btnControl_VAR_StateArr[pinN + 1] = HAL_GPIO_ReadPin(GPIOA, pinArray[pinN]);
  }
}
// # Read btn
// Docs: this function has read all btn's state. Write to btnControl_VAR_StateArr[] global array
// To protect against false alarms, a clock pass has been introduced (we register a change in state, skip clock cycles, make sure it is steady, zero it in the array)
void btnControl_Read() {
  static uint8_t pinArray[3] = {F1_Btn_Pin, F2_Btn_Pin, F3_Btn_Pin};
  static uint8_t btnChangeState[][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; // {changeRegistred, changeState, tactCounter}
  #define SKIP_TACTS 150 // max 255 (uint8_t)

  for(uint8_t pinN = 0; pinN < 3; pinN++) {
    uint8_t pinState = HAL_GPIO_ReadPin(GPIOA, pinArray[pinN]);
    if((pinState != btnChangeState[pinN][1]) && (!btnChangeState[pinN][0])) {
      btnChangeState[pinN][1] = pinState;
      btnChangeState[pinN][0] = 1;
    }
    if(btnChangeState[pinN][0]) {
      if(btnChangeState[pinN][2] <= SKIP_TACTS) {
        btnChangeState[pinN][2]++;
      } else {
        if(pinState == btnChangeState[pinN][1]) {
          btnControl_VAR_StateArr[pinN + 1] = btnChangeState[pinN][1];
          if(pinState == HIGH) { btnControl_PressEvent(pinN + 1); }
        }
        btnChangeState[pinN][0] = 0;
        btnChangeState[pinN][2] = 0;
      }
    }
  }
}

// ### Floor brd ###
// # init all I/O expanders
void floorBoard_Init() {
  uint8_t floorArray[3] = {FLOOR_1, FLOOR_2, FLOOR_3};
  uint8_t dataModeBuffer = 0x0;
  uint8_t dataStateBuffer = 0x40;
  for(uint8_t i = 0; i < 3; i++) {
    HAL_I2C_Mem_Write(&hi2c1, FLOOR_I2C_ADDR[floorArray[i] - 1]<<1, PCA_CONFIGR, I2C_MEMADD_SIZE_8BIT, &dataModeBuffer, 1, 150);  // all output
    HAL_I2C_Mem_Write(&hi2c1, FLOOR_I2C_ADDR[floorArray[i] - 1]<<1, PCA_OUTPUTR, I2C_MEMADD_SIZE_8BIT, &dataModeBuffer, 1, 150);  // all low
    HAL_I2C_Mem_Write(&hi2c1, FLOOR_I2C_ADDR[floorArray[i] - 1]<<1, PCA_OUTPUTR, I2C_MEMADD_SIZE_8BIT, &dataStateBuffer, 1, 150); // all DP
  }
}
// # save last bits (for save lamp state)
uint8_t floorBoard_BitSave(uint8_t lastFrame, uint8_t newFrame, uint8_t mode) {
  uint8_t resultByte = 0;
  // save LAMP bit state
  if(mode != FRAME_LAMP && mode != FRAME_LAMP_CLEAR && mode != FRAME_CLEAR) {
    if(lastFrame & LAMP_BIT_MASK) {
      resultByte = newFrame |= LAMP_BIT_MASK;
    } else {
      resultByte = newFrame;
    }
  } else if(mode == FRAME_LAMP) {
    resultByte = lastFrame |= LAMP_BIT_MASK;
  } else if(mode == FRAME_LAMP_CLEAR) {
    resultByte = lastFrame &= ~LAMP_BIT_MASK;
  } else {
    resultByte = newFrame;
  }
  return resultByte;
}
// # control floor brd
void floorBoard_Control(uint8_t floor, uint8_t frame) {
  uint8_t bitCombination[] = {0xa0, 0x38, 0xb0, 0x40, 0x2, 0x0, 0x0}; // PCA9534 bits combination ( 1 / 2 / 3 / DP / BTN_LAMP / CLEAR / LAMP_CLEAR)
  static uint8_t frameBuffer[] = {0x0, 0x0, 0x0}; // Frame buffer (for save last setted bits)

  // save bits and transmit data to I2C slave/s
  if(floor != FLOOR_ALL) {
    uint8_t dataBuffer = floorBoard_BitSave(frameBuffer[floor - 1], bitCombination[frame - 1], frame);
    frameBuffer[floor - 1] = dataBuffer;
    HAL_I2C_Mem_Write(&hi2c1, FLOOR_I2C_ADDR[floor - 1]<<1, PCA_OUTPUTR, I2C_MEMADD_SIZE_8BIT, &dataBuffer, 1, 150);
  } else {
    for(uint8_t i = 0; i < 3; i++) {
      uint8_t dataBuffer = floorBoard_BitSave(frameBuffer[i], bitCombination[frame - 1], frame);
      frameBuffer[i] = dataBuffer;
      HAL_I2C_Mem_Write(&hi2c1, FLOOR_I2C_ADDR[i]<<1, PCA_OUTPUTR, I2C_MEMADD_SIZE_8BIT, &dataBuffer, 1, 150);
    }
  }

}
// # Update current floor and show floor number on all floor brd
// Docs: this function has check change floor and iniciate show current floor on all floor brd's.
void updCurrentFloor() {
  static uint8_t floorFrameArray[] = {FRAME_1, FRAME_2, FRAME_3};
  for(uint8_t floorN = 0; floorN < 4; floorN++) {
    if(floorSensor_VAR_StateArr[floorN] == F_Sensor_Trigg && floorSensor_VAR_CurrentFloor != floorN) {
      floorSensor_VAR_CurrentFloor = floorN;
      if(floorN > 0) {
        floorBoard_Control(FLOOR_ALL, floorFrameArray[floorN - 1]);
      }
    }
  }
}


// ### Trip control ###
// # Check possible to start new trip
// Return: 0 - deny start a new trip / 1 - allow start a new trip
uint8_t tripControl_checkRunPossiple(uint8_t pressedFloor) {
  // Ok. We can try start new trip.
  if(!tripControl_VAR_activeTrip && !tripControl_VAR_driveLatch && floorSensor_VAR_CurrentFloor != pressedFloor) { return 1; }
  // No. We can't try start new trip.
  else { return 0; }
}
// # Start trip
// Docs: this function has set start trip flags, run drive
void tripControl_start(uint8_t assignedFloor) {
  uint8_t driveDirection = 0;

  if(assignedFloor == floorSensor_VAR_CurrentFloor) { return; }

  // select drive direct and calc floor trip count
  if(floorSensor_VAR_CurrentFloor < assignedFloor) {
    driveDirection = UP;
    tripControl_VAR_floorCount = assignedFloor - floorSensor_VAR_CurrentFloor;
  } else {
    driveDirection = DOWN;
    tripControl_VAR_floorCount = floorSensor_VAR_CurrentFloor - assignedFloor;
  }

  tripControl_VAR_activeTrip = 1;
  tripControl_VAR_assignedFloor = assignedFloor;
  // switch ON assigned floor btn lamp
  // TODO: fix
  floorBoard_Control(FLOOR_ALL, FRAME_LAMP_CLEAR);
  floorBoard_Control(assignedFloor, FRAME_LAMP);

  // start trip drive (smooth accelrate)
  motionDrive_accelerateControl(ACCELERATE, mDefaultSpeed, 65, 3);
  motionDrive_Control(START, driveDirection);
  ledIndicator_setMode(BLINK, GREEN_LED, 550);
}
// # Complete active trip
// Docs: this function has clear all trip flags, stop drive
void tripControl_complete() {
  motionDrive_Control(STOP, 0);
  ledIndicator_setMode(STATIC, GREEN_LED, 0);
  tripControl_VAR_assignedFloor = 0;
  tripControl_VAR_activeTrip = 0;
  // Switch OFF assigned floor btn lamp
  // TODO: fix
  floorBoard_Control(FLOOR_ALL, FRAME_LAMP_CLEAR);
}
// # Trip proccess supervisor
// Docs: this function has control active trip events, mode operation
void tripControl_supervisor() {
  if(tripControl_VAR_activeTrip) {
    // Event #1 - Current floor = assigned floor => complete trip (stop drive)
    if(floorSensor_VAR_CurrentFloor == tripControl_VAR_assignedFloor) { tripControl_complete(); }
    // Event #2 - Currebt floor = overfull sensor => complete trip (stop drive)
    if(floorSensor_VAR_CurrentFloor == 0) { tripControl_complete(); }
    // Event #3 - soft stop drive (drive up to assigned floor)
    //if(abs(floorSensor_VAR_CurrentFloor - tripControl_VAR_assignedFloor) == 1) {}
  }
}

// ### Events ###
// # Btn press event handler
// x1 pressed btn - try run trip? / >1 pressed btn - try stop drive?
// Not returns.
void btnControl_PressEvent(uint8_t pressedFloor) {
  if(!btnControl_VAR_pressAccess) { return; }
  // calc total pressed btn's
  uint8_t totalPressedBtns = 0;
  for(uint8_t i = 1; i < 4; i++) {
    if(btnControl_VAR_StateArr[i]) { totalPressedBtns++; }
  }
  // Events
  // #1 - Pressed x1 btn - try start trip?
  if(totalPressedBtns == 1) {
    if(tripControl_checkRunPossiple(pressedFloor) == 1) {
      tripControl_start(pressedFloor);
    }
  }
  // #2 - Pressed >1 btn -> stop drive?
  else if(totalPressedBtns > 1) { tripControl_complete(); }
}


// HAL functions
void SystemClock_Config(void) {

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00506682;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 PA4 PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line) {}
#endif /* USE_FULL_ASSERT */

