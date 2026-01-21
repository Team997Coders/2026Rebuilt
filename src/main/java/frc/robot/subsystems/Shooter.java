package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    public TalonFX flywheel1 = new TalonFX(Constants.ShooterConstants.flywheelID);
    public TalonFX flywheel2 = new TalonFX(Constants.ShooterConstants.flywheel2ID);

    public SparkMax hood = new SparkMax(Constants.ShooterConstants.hoodMotor, MotorType.kBrushless);
    public SparkMax roller = new SparkMax(Constants.ShooterConstants.rollerMotor, MotorType.kBrushless);

    public MotorOutputConfigs flywheel2Config = new MotorOutputConfigs();
    public MotorOutputConfigs flywheelConfig = new MotorOutputConfigs();

    public DigitalInput beamBreak = new DigitalInput(Constants.ShooterConstants.beamBreak);
    public DigitalInput AbsoluteEncoder = new DigitalInput(Constants.ShooterConstants.absoluteEncoder);


    public Shooter() {
 
        //could be wrong
        flywheel2Config.Inverted = InvertedValue.CounterClockwise_Positive;  
        flywheelConfig.Inverted = InvertedValue.Clockwise_Positive;  
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

}
