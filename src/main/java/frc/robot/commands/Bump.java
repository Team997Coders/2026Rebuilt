// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

import edu.wpi.first.math.controller.ProfiledPIDController;


public class Drive extends Command {

  private final Drivebase drivebase;
  private final Supplier<double[]> speedXY;
  private final DoubleSupplier rot;
  private final ProfiledPIDController thetaController = new ProfiledPIDController(1.0, 0.0, 0.0, TrapezoidProfile.Constraints(10, 10));

  /** Creates a new Drive. */
  public Drive(Drivebase drivebase, Supplier<double[]> speedXY, DoubleSupplier rot) {
    this.drivebase = drivebase;
    this.speedXY = speedXY;
    this.rot = rot;

    // Use addRequirements() here to declare subsystem dependencies.
    thetaController.setTolerance(Math.toRadians(2));

    addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var xy = speedXY.get();
    double output;
    boolean isLocking = false;
    boolean isAtAngleThreshold = false;

    if (isAtAngleThreshold == true) {
      isLocking = true;
    } else {
      isLocking = false;
    }

    if (isLocking) {
        double currentAngle = drivebase.getPose().getRotation().getRadians();
        double goalAngle = Math.toRadians(45);
        output = thetaController.calculate(currentAngle, goalAngle);
    } else {
        
        output = rot.getAsDouble
    }

    drivebase.defaultDrive(-xy[1], -xy[0], output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
