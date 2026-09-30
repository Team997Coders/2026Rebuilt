package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {
  private final DCMotorSim frontSim;
  private final DCMotorSim backSim;

  private PIDController frontController = new PIDController(elevatorSimP, elevatorSimI, elevatorSimD);
  private PIDController backController = new PIDController(elevatorSimP, elevatorSimI, elevatorSimD);
  private double elevatorFFvolts = 0.0; //Assuming FF volts is going to be the bare minimum to keep the elevator from dropping
  private double frontAppliedVolts = 0.0;
  private double backAppliedVolts = 0.0;

  private double targetHeightTicks = 0.0;
  private double targetAngleTicks = 0.0;

  public ElevatorIOSim()
  {
    frontSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(frontGearbox, 0.000036, frontMotorReduction),
            frontGearbox);
    backSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(backGearbox, 0.000036, backMotorReduction),
            backGearbox);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    frontController.setSetpoint(targetAngleTicks + targetHeightTicks);
    backController.setSetpoint(targetAngleTicks - targetHeightTicks);

    frontAppliedVolts =
          elevatorFFvolts + frontController.calculate(frontSim.getAngularPositionRad() * ticksPerRadianMotor);
    backAppliedVolts =
         -elevatorFFvolts + backController.calculate(backSim.getAngularPositionRad() * ticksPerRadianMotor);

    // Update simulation state
    frontSim.setInputVoltage(MathUtil.clamp(frontAppliedVolts, -12.0, 12.0));
    backSim.setInputVoltage(MathUtil.clamp(backAppliedVolts, -12.0, 12.0));
    frontSim.update(0.02);
    backSim.update(0.02);

    inputs.frontMotorAppliedVolts = frontAppliedVolts;
    inputs.backMotorAppliedVolts = backAppliedVolts;
    inputs.frontMotorConnected = true;
    inputs.backMotorConnected = true;
    inputs.frontMotorCurrentAmps = frontSim.getCurrentDrawAmps();
    inputs.backMotorCurrentAmps = backSim.getCurrentDrawAmps();
    inputs.frontMotorPositionRad = frontSim.getAngularPositionRad();
    inputs.backMotorPositionRad = backSim.getAngularPositionRad();
    inputs.frontMotorVelocityRadPerSec = frontSim.getAngularVelocityRadPerSec();
    inputs.backMotorVelocityRadPerSec = backSim.getAngularVelocityRadPerSec();

    inputs.targetHeightMeters = targetHeightTicks / ticksPerMeter;
    inputs.targetHeightTicks = targetHeightTicks;
    inputs.currentHeightMeters = getHeightMeters();
    inputs.currentHeightTicks = getHeightTicks();
    inputs.targetArmAngleRadians = targetAngleTicks / ticksPerRadianArm;
    inputs.targetArmAngleTicks = targetAngleTicks;
    inputs.currentArmAngleRadians = getArmAngleRadians();
    inputs.currentArmAngleTicks = getArmAngleTicks();
  }

  public void setElevatorHeightMeters(double height)
  {
    setElevatorHeightEncoderTicks(height*ticksPerMeter);
  }

  public void setElevatorHeightEncoderTicks(double ticks)
  {
    targetHeightTicks = ticks;
  }

  public void setArmAngleRad(double radians)
  {
    setArmAngleEncoderTicks(radians*ticksPerRadianArm);
  }

  public void setArmAngleEncoderTicks(double ticks)
  {
    targetAngleTicks = ticks;
  }
  
  public double getHeightMeters()
  {
    return getHeightTicks() / ticksPerMeter;
  }

  public double getHeightTicks()
  {
    return (frontSim.getAngularPositionRad() - backSim.getAngularPositionRad()) * ticksPerRadianMotor;
  }

  public double getArmAngleRadians()
  {
    return getArmAngleTicks() / ticksPerRadianArm;
  }

  public double getArmAngleTicks()
  {
    return (frontSim.getAngularPositionRad() + backSim.getAngularPositionRad()) * ticksPerRadianMotor;
  }

}

