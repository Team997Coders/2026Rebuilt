package frc.robot.subsystems;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final SparkMax spinMotor;
    private final SparkMax extendMotor;
    private final SparkMaxConfig spinConfig;
    private final SparkMaxConfig extendConfig;
                    
    private double error;
    private PIDController pid;
                         
                    
    public Intake(){
        spinMotor = new SparkMax(Constants.IntakeConstants.spinMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        extendMotor = new SparkMax(Constants.IntakeConstants.extendMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        spinConfig = new SparkMaxConfig();
        extendConfig = new SparkMaxConfig();
                    
            
        spinMotor.configure(spinConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        extendMotor.configure(extendConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            
        pid = new PIDController(Constants.IntakeConstants.p, Constants.IntakeConstants.i, Constants.IntakeConstants.d);
        error = 0;
                    
                }
            
                
            
    public void spin(double voltage) {
        spinMotor.setVoltage(voltage);
                }
            
    public void stopSpin(){
         spinMotor.setVoltage(0);
                }
            
    public void runExtendMotor(double voltage) {
        extendMotor.setVoltage(voltage);
                }
            
    public double encoderPosition(){
        return extendMotor.getEncoder().getPosition();
    }
    
    public Command intakeFuel(){
        return this.runOnce(() -> spin(Constants.IntakeConstants.spinVoltage));
    }

    public Command stopIntake(){
        return this.runOnce(() -> stopSpin());
    }

}
