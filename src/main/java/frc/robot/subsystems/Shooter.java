package frc.robot.subsystems;

import frc.robot.Constants;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    public Shooter() {
        
    }

    public SparkMax flywheel = new SparkMax(Constants.ShooterConstants.flywheelID, MotorType.kBrushless);

    public void setFlywheelVoltage(double volts) {
        flywheel.setVoltage(volts);
    }


}
