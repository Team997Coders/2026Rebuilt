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
        flywheel2.setControl(new Follower( flywheel1.getDeviceID(), MotorAlignmentValue.Aligned)); //motor alignment could be different
        flywheelConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5; //change later
        flywheelConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;
    }

    public void setFlywheelVoltage(double volts) {
        flywheel1.setVoltage(volts);
        flywheel2.setVoltage(volts);
    }

    public void setHoodMotorVoltage(double volts) {
        hood.setVoltage(volts);
    }

    public void setRollerMotorVoltage (double volts) {
        roller.setVoltage(volts);
    }

    public double getHoodPos () {
        return hoodRelativeEncoder.getPosition();
    }

    public void moveHood (double position) {
        setHoodMotorVoltage(PIDHoodController.calculate(getHoodPos(), position));
    }

    public double getFlywheel1Velocity () {
        return flywheel1.getVelocity().getValueAsDouble();
    }

    public void moveFlywheel (double velocity) {
        setFlywheelVoltage(PIDFlywheelController.calculate(getFlywheel1Velocity(), velocity));
    }

    
}
