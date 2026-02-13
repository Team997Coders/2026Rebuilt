package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

    public SparkMax hood = new SparkMax(Constants.ShooterConstants.hoodMotor, MotorType.kBrushless);
    public SparkMax roller = new SparkMax(Constants.ShooterConstants.rollerMotor, MotorType.kBrushless);

    public TalonFXConfiguration flywheel1Config = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    public TalonFXConfiguration flywheel2Config = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    public SparkMaxConfig hoodConfig = new SparkMaxConfig();

    public DigitalInput beamBreak = new DigitalInput(Constants.ShooterConstants.beamBreak);
    public Trigger beamBreakTrigger = new Trigger(() -> beamBreak.get());

    public DigitalInput magnet = new DigitalInput(Constants.ShooterConstants.magnet);

    public RelativeEncoder hoodRelativeEncoder = hood.getEncoder();

    public PIDController PIDHoodController = new PIDController(Constants.ShooterConstants.hoodPID.kp, Constants.ShooterConstants.hoodPID.ki, Constants.ShooterConstants.hoodPID.kd);

    public PIDController shooterPID = new PIDController(0.01, 0, 0);

    public SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ShooterConstants.kS, Constants.ShooterConstants.kV, Constants.ShooterConstants.kA);

    private double goalAngle;

    public Shooter() {
        shooterPID.reset();
        //could be wrong, both should be spinning the same way 
        flywheel2.setControl(new Follower(flywheel1.getDeviceID(), MotorAlignmentValue.Aligned)); //motor alignment could be different
        flywheel1Config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5; //change later
        flywheel1Config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;
        flywheel1.setNeutralMode(NeutralModeValue.Coast);
        flywheel2.setNeutralMode(NeutralModeValue.Coast);


        hoodConfig.inverted(true);
        hood.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);


        setHoodAnglePos(43.3); //angle from horizontal to top of hood 

        goalAngle = 43.3;
    }
    @Override
    public void periodic() {
        setHoodMotorVoltage(PIDHoodController.calculate(getHoodAngle(), goalAngle));

        SmartDashboard.putNumber("hood pid outpud", PIDHoodController.calculate(getHoodAngle(), goalAngle));
        SmartDashboard.putNumber("Hood angle/pos", goalAngle);
        SmartDashboard.putNumber("hood angle", getHoodAngle());
        SmartDashboard.putNumber("hood motor applied output", hood.getAppliedOutput());

        SmartDashboard.putNumber("shooter output", flywheel1.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("shooter velocity rotational", getFlywheelRotVel());
        SmartDashboard.putNumber("shooter velocity tangential", getflywheelTanVel());

        SmartDashboard.putBoolean("magnet sensor", magnet.get());
    }
    
    //Roller
    public void setRollerVoltage (double volts) {
        roller.setVoltage(volts);
    }


    //Hood

    public void setGoalAngle(double angle) {
        goalAngle = angle;
    }

    public double getHoodAngle () { //rad
        return hoodRelativeEncoder.getPosition()*360/Constants.ShooterConstants.hoodGearRatio;
    }

    public void setHoodAnglePos(double angle) {
        hoodRelativeEncoder.setPosition(angle*Constants.ShooterConstants.hoodGearRatio/360);
    }

    public void setHoodMotorVoltage(double volts) {
        hood.setVoltage(volts);
    }

    public void moveHood(double angle) {
        if (angle > Constants.ShooterConstants.hoodBottomLimit && angle < Constants.ShooterConstants.hoodTopLimit) {
            setGoalAngle(angle);
        }
    }

    public void moveHoodUpManual() {
        if(goalAngle + 1 < Constants.ShooterConstants.hoodTopLimit) {
       setGoalAngle(goalAngle+1);
        }
    }

    public void moveHoodDownManual() {
        if(goalAngle - 1 > Constants.ShooterConstants.hoodBottomLimit) {
            setGoalAngle(goalAngle-1);
        } 
    }


    //Flywheel
    private double shooterVolts = 0.0;
    public void setFlywheelVoltage(double volts) {
        shooterVolts += volts;
        flywheel1.setVoltage(-shooterVolts);
        SmartDashboard.putNumber("Flywheel Volts", shooterVolts);
    }
    
    public double getFlywheelRotVel () { //rotations/sec
        return -flywheel1.getVelocity().getValueAsDouble(); 
    }

    public double getflywheelTanVel() { //meter/sec
        return getFlywheelRotVel()*Constants.ShooterConstants.flywheelRadius; 
    }

    public void setFlywheelVelocity (double velocity) {
        // Input Velocity is in rpm == feedforward needs it in rotations/sec -> rpm/60
        double feedforwardVolts = feedforward.calculate(velocity/60);
        double vel = shooterPID.calculate(getFlywheelRotVel(), velocity/60) + feedforwardVolts;
    
        SmartDashboard.putNumber("shooter pid calculated val", vel);
        SmartDashboard.putNumber("requested velocity", velocity);
        //setFlywheelVoltage(vel);
    }

    //lil commands
    //hood
    public Command hoodUp() {
        return this.run(() -> moveHoodUpManual());
    }

    public Command hoodDown() {
        return this.run(() -> moveHoodDownManual());
    }

    //roller
    public Command moveRoller() {
        return this.run(() -> setRollerVoltage(Constants.ShooterConstants.rollerVoltage));
    }

    public Command reverseRoller() {
        return this.run(() -> setRollerVoltage(Constants.ShooterConstants.rollerReverseVoltage));
    }

    public Command stopRoller() {
        return this.runOnce(() -> setRollerVoltage(0));
    }


    //flywheel
    // public Command runFlywheel() {
    //     return this.run(() -> setFlywheelVelocity(Constants.ShooterConstants.flywheelRotationalVelocity));
    // }


    public Command runFlywheelVolt(double volts) {
        return this.run(() -> setFlywheelVoltage(volts));
    }

    // public Command reverseFlywheel() {
    //     return this.run(() -> setFlywheelVelocity(Constants.ShooterConstants.flywheelReverseVelocity));
    // }

    // public Command stopFlywheel() {
    //     return this.runOnce(() -> setFlywheelVelocity(0));
    // }

    public void moveRollerandFlywheel(double flywheelVel, double rollerVolts) {
        setFlywheelVelocity(flywheelVel);
        setRollerVoltage(rollerVolts);

    }

    public Command runRollerAndFlywheel(double flywheelVelrot, double rollerVolts) {
        return this.run(() -> moveRollerandFlywheel(flywheelVelrot, rollerVolts));
    }


    //Logging
    public void loggers() {
        SmartDashboard.putNumber("flywheel tangential Velocity", getFlywheelRotVel());
        SmartDashboard.putNumber("flywheel rotational velocity", getflywheelTanVel());

        SmartDashboard.putNumber("hood angle", getHoodAngle());
        SmartDashboard.putNumber("hood goal angle", goalAngle);
    }
}
