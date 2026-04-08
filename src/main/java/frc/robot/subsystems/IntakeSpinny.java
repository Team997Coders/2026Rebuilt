package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSpinny extends SubsystemBase {
    private final TalonFX spinMotor;
    private final TalonFXConfiguration flywheelConfig;

                    
    public IntakeSpinny(){
        spinMotor = new TalonFX(Constants.IntakeConstants.spinMotorID);
        flywheelConfig = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        flywheelConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        flywheelConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.25;
                    
    }
            
    public void spin(double voltage) {

        spinMotor.set(voltage);
    }

    public void output()
    {
        spinMotor.set(0.8);    
    }

    public Command intakeFuel()
    {
        return this.runOnce(() -> output());
    }

    public Command stopIntake()
    {
        return this.runOnce(() -> spin(0));
    }

    public Command reverse()
    {
        return this.runOnce(() -> spin(-5));
    }
}
