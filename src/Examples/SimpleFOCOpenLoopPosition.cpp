// Open loop motor control example
#include "ExampleSpace.h"

// BLDC motor & driver instance
// BLDCMotor( pp number , phase resistance)
static BLDCMotor motor = BLDCMotor(7); 
static BLDCDriver3PWM driver = BLDCDriver3PWM(5, 6, 7, 4);

//target
static float target_velocity = 0;

// instantiate the commander
static Commander command = Commander(Serial);
static void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
static void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

void EX::FocOpenLoopSetup() {
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24;
  driver.voltage_limit = 6;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // limiting motor current (provided resistance)
  motor.voltage_limit = 3;   // Volts

  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  motor.init();

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]...");

  _delay(1000);
}

void EX::FocOpenLoopLoop() {
  motor.move(target_velocity);
  // user communication
  command.run();
}