// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.KrakenUtil.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOKraken implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final TalonFX driveKraken;
  private final TalonFX turnKraken;
  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveVoltage;
  private final StatusSignal<Current> driveCurrent;
  private final StatusSignal<Voltage> turnVoltage;
  private final StatusSignal<Current> turnCurrent;
  private final Canandmag turnEncoder;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Connection debouncers
  private final Debouncer driveConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer turnConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ModuleIOKraken(int module) {
    zeroRotation =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> Rotation2d.kZero;
        };

    driveKraken =
        new TalonFX(
            switch (module) {
              case 0 -> frontLeftDriveCanId;
              case 1 -> frontRightDriveCanId;
              case 2 -> backLeftDriveCanId;
              case 3 -> backRightDriveCanId;
              default -> 0;
            });

    var driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = DriveConstants.driveGains;
    driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.driveMotorReduction;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = DriveConstants.kSlipCurrent.magnitude();
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -DriveConstants.kSlipCurrent.magnitude();
    driveConfig.CurrentLimits.StatorCurrentLimit = DriveConstants.kSlipCurrent.magnitude();
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    StatusCode configure = driveKraken.getConfigurator().apply(driveConfig, 0.25);
    if (!configure.isOK()) {
      SmartDashboard.putNumber("drive motor configuration error", configure.value);
    }
    driveKraken.setPosition(0.0, 0.25);

    turnKraken =
        new TalonFX(
            switch (module) {
              case 0 -> frontLeftTurnCanId;
              case 1 -> frontRightTurnCanId;
              case 2 -> backLeftTurnCanId;
              case 3 -> backRightTurnCanId;
              default -> 0;
            });

    var turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = DriveConstants.turnMotorGains;
    turnConfig.Feedback.FeedbackRemoteSensorID =
        switch (module) {
          case 0 -> frontLeftEncoderCanId;
          case 1 -> frontRightEncoderCanId;
          case 2 -> backLeftEncoderCanId;
          case 3 -> backRightEncoderCanId;
          default -> 0;
        };
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    turnConfig.Feedback.RotorToSensorRatio = DriveConstants.turnMotorReduction;
    turnConfig.Feedback.SensorToMechanismRatio = DriveConstants.turnMotorReduction;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / DriveConstants.turnMotorReduction;
    turnConfig.MotionMagic.MotionMagicAcceleration =
        turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * DriveConstants.turnMotorReduction;
    turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    configure = turnKraken.getConfigurator().apply(turnConfig, 0.25);
    if (!configure.isOK()) {
      SmartDashboard.putNumber("turn motor configuration error", configure.value);
    }

    turnEncoder =
        new Canandmag(
            switch (module) {
              case 0 -> frontLeftEncoderCanId;
              case 1 -> frontRightEncoderCanId;
              case 2 -> backLeftEncoderCanId;
              case 3 -> backRightEncoderCanId;
              default -> 0;
            });

    turnEncoder.setAbsPosition(0);
    configure = turnKraken.setPosition(turnEncoder.getAbsPosition());
    if (!configure.isOK()) {
      SmartDashboard.putNumber("set turn angle error", configure.value);
    }

    drivePosition = driveKraken.getPosition();
    driveVelocity = driveKraken.getVelocity();
    driveVoltage = driveKraken.getMotorVoltage();
    driveCurrent = driveKraken.getStatorCurrent();
    turnVoltage = driveKraken.getMotorVoltage();
    turnCurrent = driveKraken.getStatorCurrent();

    // Create odometry queues
    timestampQueue = KrakenOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        KrakenOdometryThread.getInstance()
            .registerSignal(driveKraken, drivePosition::getValueAsDouble);
    turnPositionQueue =
        KrakenOdometryThread.getInstance().registerSignal(turnKraken, turnEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    drivePosition.refresh();
    driveVelocity.refresh();
    driveVoltage.refresh();
    driveCurrent.refresh();

    turnVoltage.refresh();
    turnCurrent.refresh();

    // Update drive inputs
    krakenStickyFault = false;
    ifOk(driveKraken, drivePosition::getValueAsDouble, (value) -> inputs.drivePositionRad = value);
    ifOk(
        driveKraken,
        driveVelocity::getValueAsDouble,
        (value) -> inputs.driveVelocityRadPerSec = value);
    ifOk(
        driveKraken,
        new DoubleSupplier[] {driveCurrent::getValueAsDouble, driveVoltage::getValueAsDouble},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveKraken, driveCurrent::getValueAsDouble, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!krakenStickyFault);

    // Update turn inputs
    krakenStickyFault = false;
    ifOk(
        turnKraken,
        turnEncoder::getPosition,
        (value) -> inputs.turnPosition = new Rotation2d(value * 2 * Math.PI).minus(zeroRotation));
    ifOk(turnKraken, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnKraken,
        new DoubleSupplier[] {turnCurrent::getValueAsDouble, turnVoltage::getValueAsDouble},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnKraken, turnCurrent::getValueAsDouble, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!krakenStickyFault);

    SmartDashboard.putNumber("swerve current angle", turnKraken.getPosition().getValueAsDouble());

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveKraken.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnKraken.setVoltage(output);
  }

  // TODO: change out our motor controller for the FOC vector magic (+15% power and acceleration)
  // velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
  // positionTorqueCurrentRequest.withPosition(rotation.getRotations());
  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveKraken.setControl(velocityVoltageRequest.withVelocity(velocityRadPerSec));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    SmartDashboard.putNumber("swerve target angle", rotation.getDegrees());

    turnKraken.setControl(positionVoltageRequest.withPosition(rotation.getRotations()));
  }
}
