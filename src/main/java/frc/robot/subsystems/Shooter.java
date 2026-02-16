package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.vision.CameraShooter;
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
    public TalonFX flywheel1 = new TalonFX(Constants.ShooterConstants.flywheelID);
    public TalonFX flywheel2 = new TalonFX(Constants.ShooterConstants.flywheel2ID);

    public TalonFXConfiguration flywheel2Config = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    public TalonFXConfiguration flywheel1Config = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    public DigitalInput beamBreak = new DigitalInput(Constants.ShooterConstants.beamBreak);
    public Trigger beamBreakTrigger = new Trigger(() -> beamBreak.get());

    public DigitalInput magnet = new DigitalInput(Constants.ShooterConstants.magnet);

    


    public PIDController shooterPID = new PIDController(0.055, 0, 0);

    public SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(0.1, 0.11/2/Math.PI, 0);

    private PAVController pav;
    private CameraShooter shootCam;

    public Shooter(PAVController pav, CameraShooter shootCam) {
        this.shootCam = shootCam;
        this.pav = pav;
        shooterPID.reset();
        //could be wrong, both should be spinning the same way 
        flywheel2.setControl(new Follower(flywheel1.getDeviceID(), MotorAlignmentValue.Aligned)); //motor alignment could be different
        flywheel1Config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5; //change later
        flywheel1Config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;
        flywheel1.setNeutralMode(NeutralModeValue.Coast);
        flywheel2.setNeutralMode(NeutralModeValue.Coast);
    }
    @Override
    public void periodic() {


        SmartDashboard.putNumber("shooter output", flywheel1.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("shooter velocity rotational", getFlywheelRotVel());
        SmartDashboard.putNumber("shooter velocity tangential", getflywheelTanVel());
    }

    //Flywheel
    private double shooterVolts = 0.0;
    public void setFlywheelVoltage(double volts) {
        shooterVolts = volts;
        if (shooterVolts < 0) {
            shooterVolts = 0;
        }
       flywheel1.setVoltage(-shooterVolts);
       SmartDashboard.putNumber("volts", shooterVolts);

    }
    
    public double getFlywheelRotVel () { //radians/sec
        return -flywheel1.getVelocity().getValueAsDouble()*2*Math.PI; 
    }

    public double getflywheelTanVel() { //meter/sec
        return getFlywheelRotVel()*Constants.ShooterConstants.flywheelRadius; 
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
        pav.update(shootCam.getDistanceFromTarget(goalPose));
        moveFlywheel(pav.getVelocity() / Constants.ShooterConstants.flywheelRadius);
        SmartDashboard.putNumber("pav target velocity", pav.getVelocity());
    }

    public Command PAVcontrollerCommand()
    {
        return this.run(() -> PAVcontroller());
    }
}
