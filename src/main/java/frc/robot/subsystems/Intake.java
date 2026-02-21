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
    private final SparkMax extendMotorRight;
    private final SparkMax extendMotorLeft;
    private final SparkMaxConfig spinConfig;
    private final SparkMaxConfig extendConfigRight;
    private final SparkMaxConfig extendConfigLeft;

    private PIDController pid = new PIDController(Constants.IntakeConstants.p, Constants.IntakeConstants.i, Constants.IntakeConstants.d);
    private double goal = 0;    
    private RelativeEncoder encoder; 
                    
    public Intake(){
        spinMotor = new SparkMax(Constants.IntakeConstants.spinMotorID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        extendMotorRight = new SparkMax(Constants.IntakeConstants.extendMotorIDright, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        extendMotorLeft = new SparkMax(Constants.IntakeConstants.extendMotorIDleft, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        spinConfig = new SparkMaxConfig();
        extendConfigRight = new SparkMaxConfig();
        extendConfigLeft = new SparkMaxConfig();
                    
        spinMotor.configure(spinConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        extendMotorRight.configure(extendConfigRight, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        extendMotorLeft.configure(extendConfigLeft, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        encoder = extendMotorLeft.getEncoder();     
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

    public void maxStow()
    {
        extendMotorLeft.set(1);
    }

    public Command maxSpeedStow()
    {
        return this.runOnce(() -> maxStow());
    }

    public void stopExtendoMotor()
    {
        extendMotorLeft.set(0);
    }

    public Command stopExtendo()
    {
        return this.runOnce(() -> stopExtendoMotor());
    }
            
    public void spin(double voltage) {
        if (voltage > 14)
        {
            voltage = 14;
        }
        if (voltage < -4)
        {
            voltage = -4;
        }
        spinMotor.setVoltage(voltage);
    }

    public void output()
    {
        spinMotor.set(1.0);
    }
            
    public void runExtendMotor(double voltage) {
        SmartDashboard.putNumber("intake extendo voltage", voltage);
        extendMotorLeft.setVoltage(voltage);
    }

    double Kg = 8;
    public void runExtendWithGravity()
    {
        double voltage = pid.calculate(getEncoderPosition(), goal);
        if (getEncoderPosition() > goal && getEncoderPosition() < 0)
        {
            voltage += Kg;
        }
        runExtendMotor(voltage);
    }
            
    public double getEncoderPosition(){
        return encoder.getPosition();
    }
    
    public Command intakeFuel(){
        return this.runOnce(() -> spin(Constants.IntakeConstants.spinVoltage));
    }

    public Command intakeFull()
    {
        return this.runOnce(() -> output());
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

    public void toggleIntake()
    {
        if (goal == 0)
        {
            setGoal(Constants.IntakeConstants.extendedPosition);
        }
        else
        {
            setGoal(0);
        }
    }

    public Command toggleIntakeCommand()
    {
        return this.runOnce(() -> toggleIntake());
    }

    @Override
    public void periodic()
    {
        runExtendMotor(pid.calculate(getEncoderPosition(), goal));
        //runExtendWithGravity();
        SmartDashboard.putNumber("intake extension goal", goal);
        SmartDashboard.putNumber("intake extension current", getEncoderPosition());
    }
}
