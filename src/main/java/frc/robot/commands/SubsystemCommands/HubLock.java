<<<<<<< HEAD:src/main/java/frc/robot/commands/hubLock.java
// This file was left in place for compatibility during automated edits. The
// canonical class is now in `HubLock.java` (matching the public class name).
// Leave this file empty to avoid duplicate-class compile errors.
package frc.robot.commands;
=======
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SubsystemCommands;
>>>>>>> dev:src/main/java/frc/robot/commands/SubsystemCommands/HubLock.java

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;

public class hubLock extends Command {

  private final Drivebase drivebase;
  private final Supplier<double[]> speedXY;

  private final Hood m_hood;
  private final Shooter m_shooter;

  private static TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(18, 18);
  private ProfiledPIDController thetaController = new ProfiledPIDController(
      9, 2, 0, THETA_CONSTRAINTS);
  private Double[] pidValues = new Double[] { 9.0, 2.0, 0.0 };
  private double thetaTollerance = 2;
  private double Kh = 0.4;
  private boolean finished = false;

  /** Creates a new Drive. */
<<<<<<< HEAD:src/main/java/frc/robot/commands/hubLock.java
  public hubLock(Drivebase drivebase, Supplier<double[]> speedXY) {
=======
  public HubLock(Drivebase drivebase, Supplier<double[]> speedXY, Hood hood, Shooter shooter) {
>>>>>>> dev:src/main/java/frc/robot/commands/SubsystemCommands/HubLock.java
    this.drivebase = drivebase;
    this.speedXY = speedXY;

    this.m_hood = hood;
    this.m_shooter = shooter;

    thetaController.setTolerance(Units.degreesToRadians(thetaTollerance));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putNumberArray("Hub Lock PID Constants", new Double[] { 9.0, 2.0, 0.0 });

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    thetaController.reset(drivebase.getShooterPose().getRotation().getRadians());
  }

<<<<<<< HEAD:src/main/java/frc/robot/commands/hubLock.java
  public Pose2d getGoalPose() {
    if (DriverStation.getAlliance().orElseThrow().equals(DriverStation.Alliance.Red)) {
      // 10
      Pose2d tag = aprilTagFieldLayout.getTagPose(10).orElseThrow().toPose2d();
      goalPose = new Pose2d(tag.getX() - Units.inchesToMeters(47.0 / 2), tag.getY(), tag.getRotation());
    } else if (DriverStation.getAlliance().orElseThrow().equals(DriverStation.Alliance.Blue)) {
      // 26
      Pose2d tag = aprilTagFieldLayout.getTagPose(26).orElseThrow().toPose2d();
      goalPose = new Pose2d(tag.getX() + Units.inchesToMeters(47.0 / 2), tag.getY(), tag.getRotation());
    } else {
      if (drivebase.getPose().getX() > 16.53) {
        Pose2d tag = aprilTagFieldLayout.getTagPose(26).orElseThrow().toPose2d();
        goalPose = new Pose2d(tag.getX() + Units.inchesToMeters(47.0 / 2), tag.getY(), tag.getRotation());
      } else {
        Pose2d tag = aprilTagFieldLayout.getTagPose(10).orElseThrow().toPose2d();
        goalPose = new Pose2d(tag.getX() + Units.inchesToMeters(47.0 / 2), tag.getY(), tag.getRotation());
=======
  public Pose2d getGoalPose()
  {
    if (DriverStation.getAlliance().orElseThrow().equals(DriverStation.Alliance.Red))
    {
      if (drivebase.getPose().getX() < 11.901424)
      {
        goalPose = new Pose2d(drivebase.getShooterPose().getX() - 1, drivebase.getShooterPose().getY(), new Rotation2d());
      }
      else 
      {
        //10
        Pose2d tag = aprilTagFieldLayout.getTagPose(10).orElseThrow().toPose2d();
        goalPose = new Pose2d(tag.getX() - Units.inchesToMeters(47.0/2), tag.getY(), tag.getRotation());
      }
    }
    else if (DriverStation.getAlliance().orElseThrow().equals(DriverStation.Alliance.Blue))
    {
      if (drivebase.getPose().getX() > 4.611624)
      {
        goalPose = new Pose2d(drivebase.getShooterPose().getX() + 1, drivebase.getShooterPose().getY(), new Rotation2d());
      }
      else
      {
        //26
        Pose2d tag = aprilTagFieldLayout.getTagPose(26).orElseThrow().toPose2d();
        goalPose = new Pose2d(tag.getX() + Units.inchesToMeters(47.0/2), tag.getY(), tag.getRotation());
>>>>>>> dev:src/main/java/frc/robot/commands/SubsystemCommands/HubLock.java
      }
    }
    return goalPose;
  }

  public double getDistanceFromTarget(Pose2d goal) {
    return goal.getTranslation().getDistance(drivebase.getShooterPose().getTranslation());
  }

  public double getDistance() {
    return getDistanceFromTarget(getGoalPose());
  }



  private double thetaSpeed;
  private Pose2d goalPose;
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  @Override
  public void execute() {
     var xy = speedXY.get();
    double vx = drivebase.getCurrentSpeeds().vxMetersPerSecond;
    double vy = drivebase.getCurrentSpeeds().vyMetersPerSecond;
  

   

    var valuesFromSmartDashbord = SmartDashboard.getNumberArray("Hub Lock PID Constants", pidValues);
    if (!(valuesFromSmartDashbord[0].equals(pidValues[0]) && valuesFromSmartDashbord[1].equals(pidValues[1])
        && valuesFromSmartDashbord[2].equals(pidValues[2]))) {
      pidValues = valuesFromSmartDashbord;
      thetaController = new ProfiledPIDController(pidValues[0], pidValues[1], pidValues[2], THETA_CONSTRAINTS);

      thetaController.setTolerance(Units.degreesToRadians(thetaTollerance));
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
      thetaController.reset(drivebase.getShooterPose().getRotation().getRadians());
    }

    Pose2d robotPose = drivebase.getShooterPose();
    double distance = getDistanceFromTarget(robotPose);
    goalPose = getGoalPose();

<<<<<<< HEAD:src/main/java/frc/robot/commands/hubLock.java
    Pose2d robotPose = drivebase.getShooterPose();

    // thetaController.setGoal(Math.atan((goalPose.getY() - robotPose.getY() - vy *
    // Constants.airTime)
    // /(goalPose.getX() - robotPose.getX()- vx * Constants.airTime)) - Math.PI/2);
    if (DriverStation.getAlliance().orElseThrow().equals(DriverStation.Alliance.Blue)) {
      thetaController.setGoal(Math.atan((goalPose.getY() - robotPose.getY())
          / (goalPose.getX() - robotPose.getX())) - (Math.PI / 2));
    } else {
      thetaController.setGoal(Math.atan((goalPose.getY() - robotPose.getY())
          / (goalPose.getX() - robotPose.getX())) + (Math.PI / 2));
    }

    SmartDashboard.putNumber("theta goal", thetaController.getGoal().position);

    SmartDashboard.putNumber("goal: ", Math.atan((goalPose.getY() - robotPose.getY() - vy * Constants.airTime)
        / (goalPose.getX() - robotPose.getX() - vx * Constants.airTime)));
    SmartDashboard.putNumber("measered value: ", robotPose.getRotation().getRadians());
=======

    double goal = Math.atan((goalPose.getY() - robotPose.getY()) 
                        /(goalPose.getX() - robotPose.getX()));
    double shootOnMoveGoal = Math.atan(((goalPose.getY() - robotPose.getY()) + (vy * distance * Kh))
                                      /((goalPose.getX() - robotPose.getX()) + (vx * distance * Kh)));    
    


    if (DriverStation.getAlliance().orElseThrow().equals(DriverStation.Alliance.Blue))
    {
        goal -= (Math.PI/2);
        shootOnMoveGoal -= (Math.PI/2);
        
    } else
    {
        goal += (Math.PI/2);  
        shootOnMoveGoal += (Math.PI/2);
    }
    thetaController.setGoal(goal);
    
    SmartDashboard.putNumber("vy chassis speeds", vy);
    SmartDashboard.putNumber("vx chassis speeds", vx);

    SmartDashboard.putNumber("hub lock goal: ", goal);
    SmartDashboard.putNumber("hub lock shoot on move goal", shootOnMoveGoal);
    SmartDashboard.putNumber("hub lock difference (shoot on move vs. static)", goal - shootOnMoveGoal);
    SmartDashboard.putNumber("hub lock measered value: ", robotPose.getRotation().getRadians());
>>>>>>> dev:src/main/java/frc/robot/commands/SubsystemCommands/HubLock.java

    thetaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    SmartDashboard.putNumber("hub lock pid output", thetaSpeed);

<<<<<<< HEAD:src/main/java/frc/robot/commands/hubLock.java
    if (Math.abs(thetaSpeed) < 0.04 || Math.abs(Math.atan((goalPose.getY() - robotPose.getY() - vy * Constants.airTime)
        / (goalPose.getX() - robotPose.getX() - vx * Constants.airTime))
        - robotPose.getRotation().getRadians()) < 0.05) {
      thetaSpeed = 0;
    }
=======
    if (Math.abs(thetaSpeed) < 0.15) //|| //The theta speed is under 0.04 meters per second
        //(goal - robotPose.getRotation().getRadians()) < 0.05) //The goal is within 0.05 radians of the goal
    {
      thetaSpeed = 0;
    }
    SmartDashboard.putNumber("hub lock Theta speed", thetaSpeed);
>>>>>>> dev:src/main/java/frc/robot/commands/SubsystemCommands/HubLock.java

    drivebase.defaultDrive(xy[1], xy[0], thetaSpeed);

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }

  public void finish()
    {
        finished = true;
    }

  public Command finishCommand()
    {
        return Commands.runOnce(() -> finish());
    }
}