// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Bump extends Command {

  private final Drivebase m_drivebase;
  private final Supplier<double[]> speedXY;
  private final DoubleSupplier rot;
  private final ProfiledPIDController thetaController = new ProfiledPIDController(3.0, 0.0, 0.0, new TrapezoidProfile.Constraints(60, 60));
  
  public Bump(Supplier<double[]> speedXY, DoubleSupplier rot, Drivebase drivebase) {
    this.speedXY = speedXY;
    this.rot = rot;
    this.m_drivebase = drivebase;
    thetaController.setTolerance(Math.toRadians(2));

   // Use addRequirements() here to declare subsystem dependencies
    addRequirements(this.m_drivebase);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var xy = speedXY.get();
    Canandgyro gyro = m_drivebase.gyro;

    double Routput;
    double angleThreshold = 10;
    boolean isLocking = false;
    double AnglePitch = gyro.getPitch();
    double AngleRoll = gyro.getRoll();

    //detect if robot is at an angle greater than 20 degrees
    if (AnglePitch > angleThreshold || AngleRoll > angleThreshold) {

      isLocking = true;
    } else {
      
      isLocking = false;
    }

    //use isLocking boolean to turn 45 degrees if true and switch to normal drive if false
    if (isLocking == true) {

      double currentAngle = m_drivebase.getPose().getRotation().getRadians();
      double goalAngle = Math.toRadians(45);
      Routput = thetaController.calculate(currentAngle, goalAngle);
    } else {
  
        Routput = rot.getAsDouble();
    }

    //set defaultDrive to output
    m_drivebase.defaultDrive(-xy[1], -xy[0], Routput);

    SmartDashboard.putNumber("Pitch Angle", AnglePitch);
    SmartDashboard.putNumber("Roll angle", AngleRoll);
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
// :)