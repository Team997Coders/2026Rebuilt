// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SubsystemCommands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal.SignalMeasurement;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.DARE;
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
import frc.robot.subsystems.vision.PAVController;

public class ShootOnMove extends Command {

  private final Drivebase drivebase;
  private final Supplier<double[]> speedXY;

  private final Hood m_hood;
  private final Shooter m_shooter;

  private static TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(18, 18);
  private ProfiledPIDController thetaController = new ProfiledPIDController(
    9, 2, 0, THETA_CONSTRAINTS);
  private Double[] pidValues = new Double[]{9.0, 2.0, 0.0};
  private double thetaTollerance = 2;
  private double Kh = 0.4;
  private boolean finished = false;

  private PAVController m_pav;

  /** Creates a new Drive. */
  public ShootOnMove(Drivebase drivebase, Supplier<double[]> speedXY, Hood hood, Shooter shooter, PAVController pav) {
    this.drivebase = drivebase;
    this.speedXY = speedXY;

    this.m_hood = hood;
    this.m_shooter = shooter;

    this.m_pav = pav;

    thetaController.setTolerance(Units.degreesToRadians(thetaTollerance));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SmartDashboard.putNumberArray("Shoot on the move PID constants", new Double[]{9.0, 2.0, 0.0});

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    thetaController.reset(drivebase.getShooterPose().getRotation().getRadians());
  }

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
      }
    }
    return goalPose;
  }

  public double getDistanceFromTarget(Pose2d goal)
  {
    return goal.getTranslation().getDistance(drivebase.getShooterPose().getTranslation());
  }

  public double getDistance()
  {
    return getDistanceFromTarget(getGoalPose());
  }


  private double thetaSpeed;
  private Pose2d goalPose;
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public double updatingGoal = 0;

  @Override
  public void execute() {
     var xy = speedXY.get();
    double vx = drivebase.getCurrentSpeeds().vxMetersPerSecond;
    double vy = drivebase.getCurrentSpeeds().vyMetersPerSecond;


    var valuesFromSmartDashbord = SmartDashboard.getNumberArray("Shoot on the move PID constants", pidValues);
    if (!(valuesFromSmartDashbord[0].equals(pidValues[0]) && valuesFromSmartDashbord[1].equals(pidValues[1]) && valuesFromSmartDashbord[2].equals(pidValues[2])))
    {
      pidValues = valuesFromSmartDashbord;
      thetaController = new ProfiledPIDController(pidValues[0], pidValues[1], pidValues[2], THETA_CONSTRAINTS);

      thetaController.setTolerance(Units.degreesToRadians(thetaTollerance));
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
      thetaController.reset(drivebase.getShooterPose().getRotation().getRadians());
    }

    Pose2d robotPose = drivebase.getShooterPose();
    double distance = getDistance();
    goalPose = getGoalPose();

    m_pav.update(distance);

    double shootOnMoveGoal = 0;
    double shootSpeed = 0;
    double shotTime = 0;
    double hoodAngle = 25;
    double shooterVel = 0;
    
    // for (int i = 0; i < 20; i++){
      shooterVel = m_pav.getVelocity();
      hoodAngle = m_pav.getAngle();
      shootSpeed = shooterVel*Math.sin(Units.degreesToRadians(hoodAngle));
      shotTime = distance / shootSpeed;
      Translation2d robotVel = new Translation2d(vx, vy);  
      Translation2d displacement = robotVel.times(shotTime);
      Translation2d adjustedGoal = goalPose.getTranslation().plus(displacement);
      Translation2d robotToGoal = adjustedGoal.minus(robotPose.getTranslation());
      Rotation2d targetAngle = robotToGoal.getAngle();
      shootOnMoveGoal = targetAngle.getRadians();

      m_pav.update(robotToGoal.getDistance(robotPose.getTranslation()));
    //}

    SmartDashboard.putNumber("estimated shot time", shotTime);
    
    updatingGoal = shootOnMoveGoal;


    if (DriverStation.getAlliance().orElseThrow().equals(DriverStation.Alliance.Red))
    {
        shootOnMoveGoal -= (Math.PI/2);
        
    } else
    {
        shootOnMoveGoal += (Math.PI/2);
    }

    thetaController.setGoal(shootOnMoveGoal);
    
    SmartDashboard.putNumber("Shoot on move vy chassis speeds", vy);
    SmartDashboard.putNumber("Shoot on move vx chassis speeds", vx);

    SmartDashboard.putNumber("shoot on the move goal", shootOnMoveGoal);


    thetaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    SmartDashboard.putNumber("shoot on the move pid output", thetaSpeed);

    if (Math.abs(thetaSpeed) < 0.15) //|| //The theta speed is under 0.04 meters per second
        //(goal - robotPose.getRotation().getRadians()) < 0.05) //The goal is within 0.05 radians of the goal
    {
      thetaSpeed = 0;
    }
    SmartDashboard.putNumber("shoot on the move Theta speed", thetaSpeed);

    drivebase.defaultDrive(xy[1], xy[0], thetaSpeed);
    m_shooter.moveFlywheel(shooterVel / Constants.ShooterConstants.flywheelRadius);
    m_hood.setGoalAngle(hoodAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setFlywheelVoltage(0);
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