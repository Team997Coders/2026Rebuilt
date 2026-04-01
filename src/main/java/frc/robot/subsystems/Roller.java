package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    public void setRollerMotor(double setpoint) {
        roller.set(setpoint);
    }

}
