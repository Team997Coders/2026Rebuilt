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

public class Intake extends SubsystemBase {
    private final SparkMax spinMotor;
    private final SparkMax extendMotor;
    private final SparkMaxConfig spinConfig;
    private final SparkMaxConfig extendConfig;

    private PIDController pid = new PIDController(Constants.IntakeConstants.p, Constants.IntakeConstants.i, Constants.IntakeConstants.d);
    private double goal = 0;    
    private RelativeEncoder encoder; 
                    
    public Intake(){
        spinMotor = new SparkMax(Constants.IntakeConstants.spinMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        extendMotor = new SparkMax(Constants.IntakeConstants.extendMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        spinConfig = new SparkMaxConfig();
        extendConfig = new SparkMaxConfig();
                    
        spinMotor.configure(spinConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        extendMotor.configure(extendConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        encoder = extendMotor.getEncoder();     
        encoder.setPosition(0);
    }

    public void setGoal(double goal)
    {
        this.goal = goal;
    }

    public void increaseGoal()
    {
        setGoal(goal + 1);
    }

    public void decreaseGoal()
    {
        setGoal(goal - 1);
    }

    public Command increaseGoalCommand()
    {
        return this.runOnce(() -> increaseGoal());
    }

    public Command decreaseGoalCommand()
    {
        return this.runOnce(() -> decreaseGoal());
    }
            
    public void spin(double voltage) {
        spinMotor.setVoltage(voltage);
    }
            
    public void runExtendMotor(double voltage) {
        extendMotor.setVoltage(voltage);
    }
            
    public double getEncoderPosition(){
        return encoder.getPosition();
    }
    
    public Command intakeFuel(){
        return this.runOnce(() -> spin(Constants.IntakeConstants.spinVoltage));
    }

    public Command stopIntake()
    {
        return this.runOnce(() -> spin(0));
    }

    public Command extendIntake()
    {
        return this.runOnce(() -> setGoal(Constants.IntakeConstants.extendedPosition));
    }

    public Command returnIntake()
    {
        return this.runOnce(() -> setGoal(0));
    }

    @Override
    public void periodic()
    {
        runExtendMotor(pid.calculate(getEncoderPosition(), goal));
        SmartDashboard.putNumber("intake extension goal", goal);
        SmartDashboard.putNumber("intake extension current", getEncoderPosition());
    }
}
