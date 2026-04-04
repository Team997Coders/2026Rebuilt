package frc.robot.subsystems;

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
    private final SparkMax spinMotor;

    private final SparkMaxConfig spinConfig;
                    
    public IntakeSpinny(){
        spinMotor = new SparkMax(Constants.IntakeConstants.spinMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        spinConfig = new SparkMaxConfig();
                    
        spinMotor.configure(spinConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
            
    public void spin(double voltage) {

        spinMotor.set(voltage);
    }

    public void output()
    {
        spinMotor.set(-1);    
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
        return this.runOnce(() -> spin(8));
    }
}
