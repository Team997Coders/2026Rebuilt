// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.HubLock;
import frc.robot.commands.Unstick;
import frc.robot.commands.PlayMusic;
import frc.robot.commands.clumpLock;
import frc.robot.commands.goToLocation;
import frc.robot.commands.objectLock;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.ObjectCamera;
import frc.robot.subsystems.vision.PAVController;
import frc.robot.subsystems.vision.CameraBlock;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.jar.Attributes.Name;

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
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final Canandgyro gyro = new Canandgyro(Constants.gyroID);

  //The same joystick - drivestick is for joystick inputs and c_driveStick is for button triggers
  private  XboxController driveStick = new XboxController(0);
  private  CommandXboxController c_driveStick = new CommandXboxController(0);
  
  // Pathplanner autoChooser
  private SendableChooser<Command> autoChooser;

  //Cameras - pineapple is front facing camera
  //private final Camera frontCamera = new ObjectCamera("pineapple", new Transform3d(new Translation3d(0.34, 0.025, 0.013), new Rotation3d(0, 0, 0)));
  private final Camera backCamera = new Camera("backberry", new Transform3d(new Translation3d(Units.inchesToMeters(-12), Units.inchesToMeters(-2.5), Units.inchesToMeters(8)), new Rotation3d(0.0, Units.degreesToRadians(25), Math.PI)));
  private final Camera shooterCamera = new Camera("pineapple", new Transform3d(new Translation3d(Units.inchesToMeters(-11.5), Units.inchesToMeters(13.25), Units.inchesToMeters(8)), new Rotation3d(0, Units.degreesToRadians(25), Math.PI/2)));

  //private final Camera backCamera = new Camera("dragonfruit", new Transform3d(new Translation3d(-0.254, 0, 0.1524), new Rotation3d(Math.PI, -0.785, 0)));

  //Camera Block handles all cameras so we dont keep changing the amount of parameters of drivebase every time we add/remove a camera 
  private final ArrayList<Camera> cameraList = new ArrayList<Camera>(Arrays.asList(shooterCamera, backCamera));
  private final CameraBlock cameraBlock = new CameraBlock(cameraList);

  private final Drivebase drivebase = new Drivebase(gyro, cameraBlock);
  private final HubLock hubLock = new HubLock(drivebase, () -> getScaledXY());

  private final PAVController pav = new PAVController();
  private final Indexer indexer = new Indexer();
  private final Climber climber = new Climber();
  private final Shooter shooter = new Shooter(pav, hubLock);
  private final Roller roller = new Roller();
  private final Hood hood = new Hood(pav, hubLock);
  
  private Trigger unstickTrigger = new Trigger(() -> indexer.unstickFuel()) ;

  private final Unstick unstick = new Unstick(indexer);
  
  public final Intake m_intake;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
     drivebase.setDefaultCommand(
        new Drive(
             drivebase,
           () -> getScaledXY(),
           () -> scaleRotationAxis(driveStick.getRawAxis(4))));

    m_intake = new Intake();

    NamedCommands.registerCommand("move roller and index", roller.moveRoller());
    NamedCommands.registerCommand("stop roller and index", roller.stopRoller());
    //NamedCommands.registerCommand("shoot", shooter.PAVcontrollerCommand().alongWith(hood.PAVcommand()).alongWith(hubLock));
    // NamedCommands.registerCommand("move roller and index", roller.moveRoller().alongWith(indexer.startIndexer()));
    // NamedCommands.registerCommand("stop roller and index", roller.stopRoller().alongWith(indexer.stopIndexer()));
    // NamedCommands.registerCommand("shoot", shooter.PAVcontrollerCommand().alongWith(hood.PAVcommand()).alongWith(hubLock));
    // NamedCommands.registerCommand("stop shooting", shooter.moveFlywheelCommand(0));

    NamedCommands.registerCommand("extend intake", m_intake.extendIntake());
    NamedCommands.registerCommand("return intake", m_intake.returnIntake());
    NamedCommands.registerCommand("intake fuel", m_intake.intakeFuel());
    NamedCommands.registerCommand("stop intake", m_intake.stopIntake());
    NamedCommands.registerCommand("hub lock", hubLock);
    NamedCommands.registerCommand("shoot", shooter.PAVcontrollerCommand());
    NamedCommands.registerCommand("hood", hood.PAVcommand());
    NamedCommands.registerCommand("index", indexer.startIndexer());
    NamedCommands.registerCommand("stop index", indexer.stopIndexer());
    NamedCommands.registerCommand("move roller", roller.moveRoller());
    NamedCommands.registerCommand("stop roller", roller.stopRoller());

    NamedCommands.registerCommand("stop shoot", shooter.runFlywheelVolt(0));

    NamedCommands.registerCommand("raise climber", climber.raise());
    NamedCommands.registerCommand("lower climber", climber.lower());
    
    configureBindings();
    resetGyro();

    autoChooser = AutoBuilder.buildAutoChooser("moveForward");
    SmartDashboard.putData("Auto Choser", autoChooser);

    CanandEventLoop.getInstance();

    
    // NamedCommands.registerCommand("object lock set true", drivebase.setObjectLockDriveTrueCommand());
    // NamedCommands.registerCommand("object lock set false", drivebase.setObjectLockDriveFalseCommand());
    
    
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
    r = r * r * drivebase.getMaxVelocity();

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
    SmartDashboard.putNumber("Rotation", scaleRotationAxis(driveStick.getRawAxis(4)));
  }

  @SuppressWarnings("unused")
  private double cube(double input) {
    return Math.copySign(input * input * input, input);
  }

   @SuppressWarnings("unused")
   private double scaleTranslationAxis(double input) {
     return deadband(-squared(input), DriveConstants.deadband) * drivebase.getMaxVelocity();
   }

   private double scaleRotationAxis(double input) {
     return deadband(squared(input), DriveConstants.deadband) * drivebase.getMaxAngleVelocity() * -0.6;
  }

   public void resetGyro() {
    gyro.setYaw(0);
   }

   public double getGyroYaw() {
     return -gyro.getYaw();
   }

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
    //c_driveStick.leftBumper().onTrue(drivebase.setObjectLockDriveTrueCommand()).onFalse(drivebase.setObjectLockDriveFalseCommand());
    c_driveStick.rightBumper().whileTrue(m_intake.intakeFull()).onFalse(m_intake.stopIntake());

    c_driveStick.leftTrigger().whileTrue(hubLock.alongWith(shooter.PAVcontrollerCommand()).alongWith(hood.PAVcommand())).onFalse(shooter.moveFlywheelCommand(0));
    c_driveStick.rightTrigger().whileTrue(indexer.startIndexer().alongWith(roller.moveRoller())).onFalse(roller.stopRoller().alongWith(indexer.stopIndexer()));

    //c_driveStick.x().toggleOnTrue(m_intake.extendIntake()).toggleOnFalse(m_intake.returnIntake());
    c_driveStick.x().onTrue(m_intake.toggleIntakeCommand());
    c_driveStick.povRight().whileTrue(climber.climberVoltsCommand(-12));
    c_driveStick.povLeft().whileTrue(climber.climberVoltsCommand(12));
    c_driveStick.povLeft().or(c_driveStick.povRight()).whileFalse(climber.climberVoltsCommand(0));
    c_driveStick.y().whileTrue(shooter.moveFlywheelDashboardCommand()).onFalse(shooter.moveFlywheelCommand(0));
    c_driveStick.b().whileTrue(indexer.reverseIndexer()).onFalse(indexer.stopIndexer());
    c_driveStick.a().whileTrue(m_intake.reverse()).onFalse(m_intake.stopIntake());
    c_driveStick.povUp().whileTrue(hood.hoodUp());
    c_driveStick.povDown().whileTrue(hood.hoodDown());    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
