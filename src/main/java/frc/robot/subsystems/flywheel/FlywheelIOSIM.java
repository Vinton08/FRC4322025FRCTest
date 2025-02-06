// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelIOSIM extends FlywheelIOCTRE {

  private final DCMotorSim motorSimModel;
  private final TalonFXSimState leaderSim;
  private final TalonFXSimState followerSim;
  private final DCMotor motor = DCMotor.getKrakenX60Foc(2);

  public FlywheelIOSIM() {
    super();

    leaderSim = leader.getSimState();
    followerSim = follower.getSimState();

    Distance radius = Inches.of(1.5);
    double moi = Pounds.of(8.0).in(Kilograms) * Math.pow(radius.in(Meters), 2);
    LinearSystem<N2, N1, N2> linearSystem =
        LinearSystemId.createDCMotorSystem(motor, moi, GEAR_RATIO);
    motorSimModel = new DCMotorSim(linearSystem, motor);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    super.updateInputs(inputs);
    leaderSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    followerSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    double motorVoltage = leaderSim.getMotorVoltage();
    if (motorVoltage == 0) {
      motorVoltage =
          motorSimModel.getAngularVelocity().times(GEAR_RATIO).in(RadiansPerSecond)
              / motor.KvRadPerSecPerVolt;
      // add friction, this could be pulled outside the if to always apply friction
      if (motorVoltage > 0.2) {
        motorVoltage -= 0.2;
      } else if (motorVoltage < -0.2) {
        motorVoltage += 0.2;
      } else {
        motorVoltage = 0.0;
      }
    }

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    motorSimModel.setInputVoltage(motorVoltage);
    motorSimModel.update(0.020); // assume 20 ms loop time

    // Apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    leaderSim.setRotorVelocity(motorSimModel.getAngularVelocity().times(GEAR_RATIO));
    leaderSim.setRawRotorPosition(motorSimModel.getAngularPosition().times(GEAR_RATIO));
  }
}
