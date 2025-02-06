// Copyright (c) 2025 FRC 5712
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.utils.Conversions;

/**
 * CTRE-based implementation of the ElevatorIO interface for controlling an elevator mechanism. This
 * implementation uses TalonFX motors and a CANcoder for position feedback. The elevator consists of
 * a leader motor, a follower motor, and an encoder for precise positioning.
 */
public class ElevatorIOCTRE implements ElevatorIO {
  /** The gear ratio between the motor and the elevator mechanism */
  public static final double GEAR_RATIO = 2.0;

  /** The leader TalonFX motor controller (CAN ID: 30) */
  public final TalonFX leader = new TalonFX(30);
  /** The follower TalonFX motor controller (CAN ID: 31) */
  public final TalonFX follower = new TalonFX(31);

  /** The CANcoder for position feedback (CAN ID: 32) */
  public final CANcoder encoder = new CANcoder(32);

  // Status signals for monitoring motor and encoder states
  private final StatusSignal<Angle> leaderPosition = leader.getPosition();
  private final StatusSignal<Angle> leaderRotorPosition = leader.getRotorPosition();
  private final StatusSignal<AngularVelocity> leaderVelocity = leader.getVelocity();
  private final StatusSignal<AngularVelocity> leaderRotorVelocity = leader.getRotorVelocity();
  private final StatusSignal<Voltage> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Current> leaderStatorCurrent = leader.getStatorCurrent();
  private final StatusSignal<Current> followerStatorCurrent = follower.getStatorCurrent();
  private final StatusSignal<Current> leaderSupplyCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Current> followerSupplyCurrent = follower.getSupplyCurrent();
  private final StatusSignal<Angle> encoderPosition = encoder.getPosition();
  private final StatusSignal<AngularVelocity> encoderVelocity = encoder.getVelocity();

  // Debouncers for connection status (filters out brief disconnections)
  private final Debouncer leaderDebounce = new Debouncer(0.5);
  private final Debouncer followerDebounce = new Debouncer(0.5);
  private final Debouncer encoderDebounce = new Debouncer(0.5);

  /**
   * The radius of the elevator pulley/drum, used for converting between rotations and linear
   * distance
   */
  protected final Distance elevatorRadius = Inches.of(2);

  /**
   * Constructs a new ElevatorIOCTRE instance and initializes all hardware components. This includes
   * configuring both motors, setting up the follower relationship, and optimizing CAN bus
   * utilization for all devices.
   */
  public ElevatorIOCTRE() {
    // Set up follower to mirror leader
    follower.setControl(new Follower(leader.getDeviceID(), false));

    // Configure both motors with identical settings
    TalonFXConfiguration config = createMotorConfiguration();
    leader.getConfigurator().apply(config);

    // Configure update frequencies for all status signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, // 50Hz update rate
        leaderPosition,
        leaderRotorPosition,
        leaderVelocity,
        leaderRotorVelocity,
        leaderAppliedVolts,
        leaderStatorCurrent,
        followerStatorCurrent,
        leaderSupplyCurrent,
        followerSupplyCurrent,
        encoderPosition,
        encoderVelocity);

    // Optimize CAN bus usage for all devices
    leader.optimizeBusUtilization(4, 0.1);
    follower.optimizeBusUtilization(4, 0.1);
    encoder.optimizeBusUtilization(4, 0.1);
  }

  /**
   * Creates the motor configuration with appropriate settings. Sets up neutral mode, PID gains, and
   * feedback device configuration.
   *
   * @return The configured TalonFXConfiguration object
   */
  private TalonFXConfiguration createMotorConfiguration() {
    var config = new TalonFXConfiguration();
    // Set motor to coast when stopped
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Configure PID and feedforward gains
    config.Slot0.kP = 24; // Proportional gain
    config.Slot0.kI = 0; // Integral gain
    config.Slot0.kD = 1.6; // Derivative gain
    config.Slot0.kS = 0.1; // Static friction compensation
    config.Slot0.kV = 0; // Velocity feedforward
    config.Slot0.kA = 0; // Acceleration feedforward
    config.Slot0.kG = 0.7297; // Gravity feedforward

    // Use the CANcoder as the remote feedback device
    config.Feedback.withRemoteCANcoder(encoder);
    return config;
  }

  /**
   * Updates the elevator's input values with the latest sensor readings. This includes position,
   * velocity, voltage, and current measurements from both motors and the encoder, as well as
   * connection status for all devices.
   *
   * @param inputs The ElevatorIOInputs object to update with the latest values
   */
  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Refresh all sensor data
    StatusCode leaderStatus =
        BaseStatusSignal.refreshAll(
            leaderPosition,
            leaderRotorPosition,
            leaderVelocity,
            leaderRotorVelocity,
            leaderAppliedVolts,
            leaderStatorCurrent,
            leaderSupplyCurrent);

    StatusCode followerStatus =
        BaseStatusSignal.refreshAll(followerStatorCurrent, followerSupplyCurrent);

    StatusCode encoderStatus = BaseStatusSignal.refreshAll(encoderPosition, encoderVelocity);

    // Update connection status with debouncing
    inputs.leaderConnected = leaderDebounce.calculate(leaderStatus.isOK());
    inputs.followerConnected = followerDebounce.calculate(followerStatus.isOK());
    inputs.encoderConnected = encoderDebounce.calculate(encoderStatus.isOK());

    // Update position and velocity measurements
    inputs.leaderPosition = leaderPosition.getValue();
    inputs.leaderRotorPosition = leaderRotorPosition.getValue();
    inputs.leaderVelocity = leaderVelocity.getValue();
    inputs.leaderRotorVelocity = leaderRotorVelocity.getValue();

    inputs.encoderPosition = encoderPosition.getValue();
    inputs.encoderVelocity = encoderVelocity.getValue();

    // Update voltage and current measurements
    inputs.appliedVoltage = leaderAppliedVolts.getValue();
    inputs.leaderStatorCurrent = leaderStatorCurrent.getValue();
    inputs.followerStatorCurrent = followerStatorCurrent.getValue();
    inputs.leaderSupplyCurrent = leaderSupplyCurrent.getValue();
    inputs.followerSupplyCurrent = followerSupplyCurrent.getValue();

    // Calculate actual elevator distance using encoder position
    // Note: Using gear ratio of 1 since encoder rotations match elevator movement
    inputs.elevatorDistance =
        Conversions.rotationsToMeters(inputs.encoderPosition, 1, elevatorRadius);
  }

  /**
   * Sets the desired distance for the elevator to move to. Converts the desired linear distance to
   * encoder rotations and applies position control.
   *
   * @param distance The target distance for the elevator
   */
  @Override
  public void setDistance(Distance distance) {
    // Convert desired distance to rotations and set position control
    leader.setControl(
        new PositionVoltage(Conversions.metersToRotations(distance, 1, elevatorRadius)));
  }

  /**
   * Stops all elevator movement by stopping the leader motor. The follower will also stop due to
   * the follower relationship.
   */
  @Override
  public void stop() {
    leader.stopMotor();
  }
}
