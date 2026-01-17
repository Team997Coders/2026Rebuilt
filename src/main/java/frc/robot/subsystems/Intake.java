package frc.robot.subsystems;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final SparkMax spinMotor;
    private final SparkMax extendMotor;

    public Intake(){
        spinMotor = new SparkMax(Constants.IntakeConstants.spinMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        extendMotor = new SparkMax(Constants.IntakeConstants.extendMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    }

    public void spin(double voltage) {
      spinMotor.setVoltage(voltage);
    }

    public void stopIntake(){
        spinMotor.setVoltage(0);
    }
}
