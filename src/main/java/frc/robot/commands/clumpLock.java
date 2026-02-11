// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal.SignalMeasurement;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.DARE;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.vision.ObjectCamera;

public class clumpLock extends Command {

  private final Drivebase drivebase;
  private final Supplier<double[]> speedXY;

  private static TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(18, 18);
  private ProfiledPIDController thetaController = new ProfiledPIDController(
    9, 2, 0, THETA_CONSTRAINTS);
  private Double[] pidValues = new Double[]{6.0, 3.0, 1.0};
  private double thetaTollerance = 5;
  private ObjectCamera camera;
    

  /** Creates a new Drive. */
  public clumpLock(Drivebase drivebase, Supplier<double[]> speedXY, ObjectCamera camera) {
    this.drivebase = drivebase;
    this.speedXY = speedXY;
    this.camera = camera;

    thetaController.setTolerance(Units.degreesToRadians(thetaTollerance));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putNumberArray("clump Lock PID Constants", new Double[]{6.0, 3.0, 1.0});

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset(drivebase.getFieldAngle()*2*Math.PI);
    yaw = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  private double thetaSpeed;
  private boolean yaw = false;
  @Override
  public void execute() {
    var xy = speedXY.get();

    var valuesFromSmartDashbord = SmartDashboard.getNumberArray("clump Lock PID Constants", pidValues);
    if (!(valuesFromSmartDashbord[0].equals(pidValues[0]) && valuesFromSmartDashbord[1].equals(pidValues[1]) && valuesFromSmartDashbord[2].equals(pidValues[2])))
    {
      yaw = false;
      pidValues = valuesFromSmartDashbord;
      thetaController = new ProfiledPIDController(pidValues[0], pidValues[1], pidValues[2], THETA_CONSTRAINTS);

      thetaController.setTolerance(Units.degreesToRadians(thetaTollerance));
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
      thetaController.reset(drivebase.getFieldAngle()*2*Math.PI);
    }

    double targetYaw = camera.getYawClump();
    SmartDashboard.putNumber("target yaw", targetYaw);

    if (targetYaw != Double.MAX_VALUE)
    {
      //10
      yaw = true;
      thetaController.setGoal(targetYaw/180*Math.PI + drivebase.getFieldAngle()*2*Math.PI);
      SmartDashboard.putNumber("theta controller goal", targetYaw/180*Math.PI + drivebase.getFieldAngle()*2*Math.PI);
      thetaSpeed = thetaController.calculate(drivebase.getFieldAngle()*2*Math.PI);
      SmartDashboard.putNumber("theta speed", thetaSpeed);

      if (Math.abs(Math.abs(thetaController.getGoal().position)-Math.abs(drivebase.getFieldAngle()*2*Math.PI)) < Units.degreesToRadians(thetaTollerance))
      {
        thetaSpeed = 0;
      }

      drivebase.defaultDrive(-xy[1], -xy[0], -thetaSpeed);
    }
    else if (yaw)
    {
      thetaSpeed = thetaController.calculate(drivebase.getFieldAngle()*2*Math.PI);
      SmartDashboard.putNumber("theta speed", thetaSpeed);

      if (Math.abs(Math.abs(thetaController.getGoal().position)-Math.abs(drivebase.getFieldAngle()*2*Math.PI)) < Units.degreesToRadians(thetaTollerance))
      {
        thetaSpeed = 0;
      }

      drivebase.defaultDrive(-xy[1], -xy[0], -thetaSpeed);
    }
    else 
    {
      drivebase.defaultDrive(-xy[1], -xy[0], 0);
    }  
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