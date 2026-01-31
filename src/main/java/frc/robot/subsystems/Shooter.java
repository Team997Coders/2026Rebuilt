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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    
    //make sure to ramp up the wheels when running
    public TalonFX flywheel1 = new TalonFX(Constants.ShooterConstants.flywheelID);
    public TalonFX flywheel2 = new TalonFX(Constants.ShooterConstants.flywheel2ID);

    public SparkMax hood = new SparkMax(Constants.ShooterConstants.hoodMotor, MotorType.kBrushless);
    public SparkMax roller = new SparkMax(Constants.ShooterConstants.rollerMotor, MotorType.kBrushless);

    public TalonFXConfiguration flywheel2Config = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    public TalonFXConfiguration flywheelConfig = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    public DigitalInput beamBreak = new DigitalInput(Constants.ShooterConstants.beamBreak);
    public RelativeEncoder hoodRelativeEncoder = hood.getEncoder();

    public PIDController PIDFlywheelController = new PIDController(Constants.ShooterConstants.kp, Constants.ShooterConstants.ki, Constants.ShooterConstants.kd);
    public PIDController PIDHoodController = new PIDController(Constants.ShooterConstants.kp, Constants.ShooterConstants.ki, Constants.ShooterConstants.kd);

    public Shooter() {
        //could be wrong, both should be spinning the same way 
        flywheel2.setControl(new Follower(flywheel1.getDeviceID(), MotorAlignmentValue.Aligned)); //motor alignment could be different
        flywheelConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5; //change later
        flywheelConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;
    }

    double goalAngle;
    @Override
    public void periodic() {
        moveHood(goalAngle);
    }

    
    //Roller
    public void setRollerVoltage (double volts) {
        roller.setVoltage(volts);
    }


    //Hood
    public double getHoodPos () {
        return hoodRelativeEncoder.getPosition();
    }

    public void setHoodMotorVoltage(double volts) {
        hood.setVoltage(volts);
    }

    public void moveHood(double angle) {
        goalAngle = angle;
        setHoodMotorVoltage(PIDHoodController.calculate(getHoodPos(), goalAngle));
    }

    public void moveHoodUpManual() {
        goalAngle = getHoodPos() + 1;
    }

    public void moveHoodDownManual() {
        goalAngle = getHoodPos() - 1;
    }


    //Flywheel
      public void setFlywheelVoltage(double volts) {
        flywheel1.setVoltage(volts);
        flywheel2.setVoltage(volts);
    }

    public double getFlywheelVelocity () {
        return flywheel1.getVelocity().getValueAsDouble();
    }

    public void setFlywheelVelocity (double velocity) {
        setFlywheelVoltage(PIDFlywheelController.calculate(getFlywheelVelocity(), velocity));
    }


    //lil commands
    public Command hoodUp() {
        return this.runOnce(() -> moveHoodUpManual());
    }

    public Command hoodDown() {
        return this.runOnce(() -> moveHoodDownManual());
    }

    public Command moveRoller() {
        return this.run(() -> setRollerVoltage(Constants.ShooterConstants.rollerVoltage));
    }

    public Command stopRoller() {
        return this.runOnce(() -> setRollerVoltage(0));
    }

    public Command runFlywheel() {
        return this.run(() -> setFlywheelVelocity(Constants.ShooterConstants.flywheelVelocity));
    }

    public Command stopFlywheel() {
        return this.runOnce(() -> setFlywheelVelocity(0));
    }

}
