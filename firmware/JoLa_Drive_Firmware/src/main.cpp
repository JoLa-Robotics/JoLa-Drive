/** 
 * B-G431B-ESC1 velocity motion control example with an AS5600 I2C sensor
 * Retrieved from Simple FOC Examples 
 * https://docs.simplefoc.com/magnetic_sensor_i2c
 *
 */
#include <SimpleFOC.h>
#include <Arduino.h>
#include <Wire.h>

// Motor instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
// Gain calculation shown at https://community.simplefoc.com/t/b-g431b-esc1-current-control/521/21
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);


// AS5600 12-bit magnetic sensor at I2C address 0x36
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.motion(&motor, cmd); }

void setup() {
  
  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // The B-G431B-ESC1 Arduino variant maps I2C1 to PB7 (SDA) and PB8 (SCL).
  Wire.setSDA(PB7);
  Wire.setSCL(PB8);
  Wire.setClock(400000);
  Wire.begin();

  // initialize the AS5600 sensor hardware
  sensor.init(&Wire);

  // link the motor to the sensor
  motor.linkSensor(&sensor);
  
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  // link current sense and the driver
  currentSense.linkDriver(&driver);

  // current sensing
  currentSense.init();
  // no need for aligning
  currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);

  // aligning voltage [V]
  motor.voltage_sensor_align = 3;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;

  // contoller configuration 
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.5;
  motor.PID_velocity.I = 0;
  // default voltage_power_supply
  motor.voltage_limit = 6;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
 
  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01;

  // maximal velocity command
  motor.velocity_limit = 4;

  // comment out if not needed
  motor.useMonitoring(Serial);
  
  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target velocity");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity in rad/s using serial terminal:"));
  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  // motor.monitor();
  
  // user communication
  command.run();
}