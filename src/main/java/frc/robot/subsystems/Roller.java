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

public class Roller extends SubsystemBase {
    
    //make sure to ramp up the wheels when running
    public SparkMax roller = new SparkMax(Constants.ShooterConstants.rollerMotor, MotorType.kBrushless);

    public Roller() {}

    @Override
    public void periodic() {
    }
    
    //Roller
    public void setRollerVoltage (double volts) {
        roller.setVoltage(volts);
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

}
