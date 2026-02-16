package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.HubLock;
import frc.robot.commands.HubLock;
import frc.robot.subsystems.vision.PAVController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import java.net.ContentHandler;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Shooter extends SubsystemBase {
    
    //make sure to ramp up the wheels when running
    private TalonFX flywheel1 = new TalonFX(Constants.ShooterConstants.flywheelID);
    private TalonFX flywheel2 = new TalonFX(Constants.ShooterConstants.flywheel2ID);

    private TalonFXConfiguration flywheel2Config = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    private TalonFXConfiguration flywheel1Config = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    private DigitalInput beamBreak = new DigitalInput(Constants.ShooterConstants.beamBreak);
    private Trigger beamBreakTrigger = new Trigger(() -> beamBreak.get());

    private DigitalInput magnet = new DigitalInput(Constants.ShooterConstants.magnet);
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    private PIDController shooterPID = new PIDController(0.055, 0, 0);
    private SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(0.1, 0.11/2/Math.PI, 0);

    private PAVController pav;
    private HubLock hubLock;

<<<<<<< HEAD
    public Shooter(PAVController pav, HubLock hubLock) {
        this.hubLock = hubLock;
=======
    /**
     * The shooter subsystem of the robot, contains both flywheels and velocity control for shooter
     * @param pav Position Angle Velocity Contoller, used for shooter configuration given the distance from the hub
     * @param shootCam The temporary SwerveDrivePoseEstimator without the swerve, just uses camera for localization.
     * We used this to get the actual distance from the hub and update the location on the field.
     */
    public Shooter(PAVController pav, CameraShooter shootCam) {
        this.shootCam = shootCam;
>>>>>>> ShooterAngleTesting
        this.pav = pav;
        shooterPID.reset();
        flywheel2.setControl(new Follower(flywheel1.getDeviceID(), MotorAlignmentValue.Aligned)); 
        flywheel1Config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5;
        flywheel1Config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;
        flywheel1.setNeutralMode(NeutralModeValue.Coast);
        flywheel2.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Periodically updates the dashboard with the flywheel output, tangential velocity, and rotational velocity
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter output", flywheel1.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("shooter velocity rotational", getFlywheelRotVel());
        SmartDashboard.putNumber("shooter velocity tangential", getflywheelTanVel());
    }

    /**
     * Sets the voltage of the flywheels.
     * Limits the voltage to only positive numbers. 
     * (Dont run the flywheel backwards)
     */
    private double shooterVolts = 0.0;
    public void setFlywheelVoltage(double volts) {
        shooterVolts = volts;

        if (shooterVolts < 0) {
            shooterVolts = 0;
        }

       flywheel1.setVoltage(-shooterVolts);

       SmartDashboard.putNumber("volts", shooterVolts);
    }
    
    /**
     * @return the current rotational velocity of the flywheels in radians/second
     */
    public double getFlywheelRotVel () { 
        return -flywheel1.getVelocity().getValueAsDouble()*2*Math.PI; 
    }

    /**
     * @return the current tangential velocity of the flywheels in meters/second
     */
    public double getflywheelTanVel() { 
        return getFlywheelRotVel()*Constants.ShooterConstants.flywheelRadius; 
    }

    /**
     * Flywheel velocity is in RADIANS/SECOND
     * @param flywheelVel
     * Sets the rotational velocity (omega) of the shooter to requested speed using pid and ff
     */
    public void moveFlywheel(double flywheelVel) {
        double voltage = shooterPID.calculate(getFlywheelRotVel(), flywheelVel);
        double ff = shooterFF.calculate(flywheelVel);
       
        SmartDashboard.putNumber("shooter pid calculated volts", voltage);

        voltage += ff;

        SmartDashboard.putNumber("ff value", ff);
        SmartDashboard.putNumber("actual set voltage", voltage);
        SmartDashboard.putNumber("requested velocity", flywheelVel);

        setFlywheelVoltage(voltage);
    }

    /**
     * This method runs the flywheels with the TANGENTIAL velocity (in meters/second) specified on Smart Dashboard.
     * Change the "shooter velocity setpoint" on elastic to set specified velocity
     */
    public void flywheelWithDashboard()
    {
        moveFlywheel(SmartDashboard.getNumber("shooter velocity setpoint", 0.0) / Constants.ShooterConstants.flywheelRadius);
    }

    /**
     * This method runs the flywheel using the position angle velocity contoller
     * It first updates the controller target angle + velocity by passing the distance from the target 
     * Then sets the flywheels to go to correct tangential velocity
     */
    public void PAVcontroller() {
        Pose2d tag = aprilTagFieldLayout.getTagPose(10).orElseThrow().toPose2d();
        Pose2d goalPose = new Pose2d(tag.getX() - Units.inchesToMeters(47.0/2), tag.getY(), tag.getRotation());
        pav.update(shootCam.getDistanceFromTarget(goalPose));
        moveFlywheel(pav.getVelocity() / Constants.ShooterConstants.flywheelRadius);
        SmartDashboard.putNumber("pav target velocity", pav.getVelocity());
    }

    
    public Command runFlywheelVolt(double volts) {
        return this.run(() -> setFlywheelVoltage(volts));
    }

    public Command moveFlywheelCommand(double velocity)
    {
        return this.run(() -> moveFlywheel(velocity));
    }

    public Command moveFlywheelDashboardCommand()
    {
        return this.run(() -> flywheelWithDashboard());
    }

<<<<<<< HEAD
    public void moveFlywheel(double flywheelVel) {
        double vel = shooterPID.calculate(getFlywheelRotVel(), flywheelVel); //+ flywheelVel*Constants.ShooterConstants.kf;
        double ff = shooterFF.calculate(flywheelVel);
        SmartDashboard.putNumber("ff value", ff);
        SmartDashboard.putNumber("shooter pid calculated val", vel);
        vel += ff;
        SmartDashboard.putNumber("actual set vel", vel);
        SmartDashboard.putNumber("requested velocity", flywheelVel);

        setFlywheelVoltage(vel);
    }

    public void flywheelWithDashboard()
    {
        moveFlywheel(SmartDashboard.getNumber("shooter velocity setpoint", 0.0) / Constants.ShooterConstants.flywheelRadius);
    }


    //Logging
    public void loggers() {
        SmartDashboard.putNumber("flywheel tangential Velocity", getFlywheelRotVel());
        SmartDashboard.putNumber("flywheel rotational velocity", getflywheelTanVel());

    }

    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);


    public void PAVcontroller() {
        Pose2d tag = aprilTagFieldLayout.getTagPose(10).orElseThrow().toPose2d();
        Pose2d goalPose = new Pose2d(tag.getX() - Units.inchesToMeters(47.0/2), tag.getY(), tag.getRotation());
        pav.update(hubLock.getDistanceFromTarget(goalPose));
        moveFlywheel(pav.getVelocity() / Constants.ShooterConstants.flywheelRadius);
        SmartDashboard.putNumber("pav target velocity", pav.getVelocity());
    }

=======
>>>>>>> ShooterAngleTesting
    public Command PAVcontrollerCommand()
    {
        return this.run(() -> PAVcontroller());
    }
}
