package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
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

    public TalonFXConfiguration flywheel2Config = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    public TalonFXConfiguration flywheel1Config = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    public SparkMaxConfig hoodConfig = new SparkMaxConfig();

    public DigitalInput beamBreak = new DigitalInput(Constants.ShooterConstants.beamBreak);
    public Trigger beamBreakTrigger = new Trigger(() -> beamBreak.get());

    public RelativeEncoder hoodRelativeEncoder = hood.getEncoder();

    public PIDController PIDFlywheelController = new PIDController(Constants.ShooterConstants.flywheelPID.kp, Constants.ShooterConstants.flywheelPID.ki, Constants.ShooterConstants.flywheelPID.kd);
    public PIDController PIDHoodController = new PIDController(Constants.ShooterConstants.hoodPID.kp, Constants.ShooterConstants.hoodPID.ki, Constants.ShooterConstants.hoodPID.kd);

    public Shooter() {
        //could be wrong, both should be spinning the same way 
        flywheel2.setControl(new Follower(flywheel1.getDeviceID(), MotorAlignmentValue.Aligned)); //motor alignment could be different
        flywheel1Config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5; //change later
        flywheel1Config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;

        hoodConfig.inverted(true);
        hood.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    }

    double goalAngle;
    @Override
    public void periodic() {
        setHoodMotorVoltage(PIDHoodController.calculate(getHoodAngle(), goalAngle));
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
        return hoodRelativeEncoder.getPosition()*2*Math.PI;
    }

    public void setHoodMotorVoltage(double volts) {
        hood.setVoltage(volts);
    }

    public void moveHood(double angle) {
        if(angle > Constants.ShooterConstants.hoodBottomLimit && angle < Constants.ShooterConstants.hoodTopLimit) {
            setGoalAngle(angle);
        }
        

    }

    public void moveHoodUpManual() {
        if(goalAngle + 1 < Constants.ShooterConstants.hoodTopLimit) {
       setGoalAngle(goalAngle+3);
        }
    }

    public void moveHoodDownManual() {
        if(goalAngle - 1 > Constants.ShooterConstants.hoodBottomLimit) {
        setGoalAngle(goalAngle-3);
    } 
    }


    //Flywheel
      public void setFlywheelVoltage(double volts) {
        flywheel1.setVoltage(-volts);
        flywheel2.setVoltage(-volts);
    }

    public double getFlywheelRotationalVelocity () { //rad/sec
        return flywheel1.getVelocity().getValueAsDouble()*(2*Math.PI)*(Constants.ShooterConstants.flywheelGearRatio); 
    }

    public double flywheelTangentialVelocity() { //meter/sec
        return getFlywheelRotationalVelocity()*Constants.ShooterConstants.flywheelRadius; 
    }

    public void setFlywheelVelocity (double velocity) {
        setFlywheelVoltage(PIDFlywheelController.calculate(getFlywheelRotationalVelocity(), velocity));
    }


    //lil commands
    public Command hoodUp() {
        return this.run(() -> moveHoodUpManual());
    }

    public Command hoodDown() {
        return this.run(() -> moveHoodDownManual());
    }

    public Command moveRoller() {
        return this.run(() -> setRollerVoltage(Constants.ShooterConstants.rollerVoltage));
    }

    public Command reverseRoller() {
        return this.run(() -> setRollerVoltage(Constants.ShooterConstants.rollerReverseVoltage));
    }

    public Command stopRoller() {
        return this.runOnce(() -> setRollerVoltage(0));
    }

    public Command runFlywheel() {
        return this.run(() -> setFlywheelVelocity(Constants.ShooterConstants.flywheelVelocity));
    }


    public Command runFlywheelVolt(double volts) {
        return this.run(() -> setFlywheelVoltage(volts));
    }

    public Command reverseFlywheel() {
        return this.run(() -> setFlywheelVelocity(Constants.ShooterConstants.flywheelReverseVelocity));
    }

    public Command stopFlywheel() {
        return this.runOnce(() -> setFlywheelVelocity(0));
    }

    public void moveRollerandFlywheel(double FlywheeVolts, double rollerVolts) {
        setFlywheelVoltage(FlywheeVolts);
        setRollerVoltage(rollerVolts);
    }

    public Command runRollerAndFlywheel(double flywheelVolts, double rollerVolts) {
        return this.run(() -> moveRollerandFlywheel(flywheelVolts, rollerVolts));
    }


    //Logging
    public void loggers() {
        SmartDashboard.putNumber("flywheel tangential Velocity", flywheelTangentialVelocity());
        SmartDashboard.putNumber("flywheel rotational velocity", getFlywheelRotationalVelocity());

        SmartDashboard.putNumber("hood angle", getHoodAngle());
        SmartDashboard.putNumber("hood goal angle", goalAngle);
    }
}
