// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.HubLock;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Shoot;
import frc.robot.commands.Unstick;
import frc.robot.commands.PlayMusic;
import frc.robot.commands.clumpLock;
import frc.robot.commands.objectLock;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.ObjectCamera;
import frc.robot.subsystems.vision.CameraBlock;
import frc.robot.subsystems.vision.CameraShooter;

import java.util.ArrayList;
import java.util.Arrays;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.canand.CanandEventLoop;
import com.reduxrobotics.sensors.canandcolor.DigoutChannel.Index;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 // private final Canandgyro gyro = new Canandgyro(Constants.gyroID);

  //The same joystick - drivestick is for joystick inputs and c_driveStick is for button triggers
  private static XboxController driveStick = new XboxController(0);
  private static CommandXboxController c_driveStick = new CommandXboxController(0);
  
 // private static HubLock hubLock;

 // private static Shoot shootCommand;

  //Pathplanner autoChooser
  //private SendableChooser<Command> autoChooser;

  //Cameras - pineapple is front facing camera
  //private static final ObjectCamera frontCamera = new ObjectCamera("pineapple", new Transform3d(new Translation3d(0.34, 0.025, 0.013), new Rotation3d(0, 0, 0)));
  private static final CameraShooter leftCamera = new CameraShooter("blueberry", new Transform3d(new Translation3d(0.0, 0,0), new Rotation3d(Units.degreesToRadians(0), 0.0, 0)));

  //private static final Camera backCamera = new Camera("dragonfruit", new Transform3d(new Translation3d(-0.254, 0, 0.1524), new Rotation3d(Math.PI, -0.785, 0)));

  //Camera Block handles all cameras so we dont keep changing the amount of parameters of drivebase every time we add/remove a camera 
 // private static final ArrayList<Camera> cameraList = new ArrayList<CameraList>(Arrays.asList(frontCamera, leftCamera));
 // private static final CameraBlock cameraBlock = new CameraBlock(cameraList);

 // private final Drivebase drivebase = new Drivebase(gyro, cameraBlock);
  
 private static final Indexer indexer = new Indexer();
  private static final Shooter shooter = new Shooter();

 // private Trigger unstickTrigger = new Trigger(() ->indexer.unstickFuel()) ;

 // private final Unstick unstick = new Unstick(indexer);

  //
  private final ArrayList<Pose2d> potentialLocations = new ArrayList<Pose2d>();

  // public final Intake m_intake;
  // public final IntakeCommand IntakeCommandExtend;
  // public final IntakeCommand IntakeCommandRetract;
  public final Shoot ShootCommand;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

  // shootCommand = new Shoot(shooter, indexer);
    // Configure the trigger bindings
    // drivebase.setDefaultCommand(
    //     new Drive(
    //         drivebase,
    //         () -> getScaledXY(),
    //         () -> scaleRotationAxis(driveStick.getRawAxis(4))));

   // autoChooser = AutoBuilder.buildAutoChooser("moveForward");
   // SmartDashboard.putData("Auto Choser", autoChooser);

    CanandEventLoop.getInstance();

  

    // m_intake = new Intake();
    // IntakeCommandExtend = new IntakeCommand(m_intake, true);
    // IntakeCommandRetract = new IntakeCommand(m_intake, false);
 
      ShootCommand = new Shoot(shooter, indexer, leftCamera);

    //NamedCommands.registerCommand("object lock set true", drivebase.setObjectLockDriveTrueCommand());
    //NamedCommands.registerCommand("object lock set false", drivebase.setObjectLockDriveFalseCommand());
    
    configureBindings();
  }

  /**
   * {@link edu.wpi.first.math.MathUtil}
   */
  private double deadband(double input, double deadband) {
    if (Math.abs(input) < deadband) {
      return 0;
    } else {
      return input;
    }
  }

  private double[] getXY() {
    double[] xy = new double[2];
    xy[0] = deadband(driveStick.getLeftX(), DriveConstants.deadband);
    xy[1] = deadband(driveStick.getLeftY(), DriveConstants.deadband);
    return xy;
  }

  private double[] getScaledXY() {
    double[] xy = getXY();

    // Convert to Polar coordinates
    double r = Math.sqrt(xy[0] * xy[0] + xy[1] * xy[1]);
    double theta = Math.atan2(xy[1], xy[0]);

    // Square radius and scale by max velocity
   // r = r * r * drivebase.getMaxVelocity();

    // Convert to Cartesian coordinates
    xy[0] = r * Math.cos(theta);
    xy[1] = r * Math.sin(theta);

    return xy;
  }

  private double squared(double input) {
    return Math.copySign(input * input, input);
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Scaled_X", getScaledXY()[0]);
    SmartDashboard.putNumber("Scaled_Y", getScaledXY()[1]);
   // SmartDashboard.putNumber("Rotation", scaleRotationAxis(driveStick.getRawAxis(4)));
  }

  @SuppressWarnings("unused")
  private double cube(double input) {
    return Math.copySign(input * input * input, input);
  }

  // @SuppressWarnings("unused")
  // private double scaleTranslationAxis(double input) {
  //   return deadband(-squared(input), DriveConstants.deadband) * drivebase.getMaxVelocity();
  // }

  // private double scaleRotationAxis(double input) {
  //   return deadband(squared(input), DriveConstants.deadband) * drivebase.getMaxAngleVelocity() * -0.6;
  // }

  // public void resetGyro() {
  //   gyro.setYaw(0);
  // }

  // public double getGyroYaw() {
  //   return -gyro.getYaw();
  // }

  public boolean onBlueAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == Alliance.Blue;
    }
    return false;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //c_driveStick.rightTrigger().whileTrue(ShootCommand);
   // c_driveStick.rightTrigger().whileFalse(shooter.runRollerAndFlywheel(0, 0));
   // c_driveStick.b().whileTrue(shootCommand);

   c_driveStick.rightTrigger().whileTrue(ShootCommand);
  c_driveStick.leftTrigger().whileTrue(shooter.runRollerAndFlywheel(Constants.ShooterConstants.flywheelVoltage, Constants.ShooterConstants.rollerVoltage));
  c_driveStick.a().onTrue(shooter.moveRoller()).onFalse(shooter.stopRoller());
  c_driveStick.leftTrigger().whileFalse(shooter.runRollerAndFlywheel(0,0));
    c_driveStick.povUp().whileTrue(shooter.hoodUp());
    c_driveStick.povDown().whileTrue((shooter.hoodDown()));

    



   // c_driveStick.a().onTrue(shooter.runFlywheelVolt(5));

  
    // c_driveStick.leftBumper().onTrue(IntakeCommandExtend);
    // c_driveStick.rightBumper().onTrue(IntakeCommandRetract);
  // c_driveStick.a().whileTrue(m_intake.intakeFuel());
    // Gyro Reset
    //c_driveStick.povUp().onTrue(Commands.runOnce(gyro::reset));
    
    //When holding x robot goes to closest location in potential locations
    //c_driveStick.x().whileTrue(new goToLocation(drivebase, potentialLocations));
    // c_driveStick.b().whileTrue(new objectLock(drivebase, () -> getScaledXY(), frontCamera));
    // c_driveStick.a().whileTrue(new clumpLock(drivebase, () -> getScaledXY(), frontCamera));
    // c_driveStick.x().whileTrue(new PlayMusic(drivebase));
    // c_driveStick.leftBumper().onTrue(IntakeCommandExtend);
    // c_driveStick.rightBumper().onTrue(IntakeCommandRetract);
    // c_driveStick.y().whileTrue(m_intake.intakeFuel());
    



    //c_driveStick.rightTrigger().whileTrue(indexer.startIndexer());

   // unstickTrigger.whileTrue(unstick);


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
