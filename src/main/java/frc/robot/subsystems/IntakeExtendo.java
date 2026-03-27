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

public class IntakeExtendo extends SubsystemBase {
    private final SparkMax extendMotorRight;
    private final SparkMax extendMotorLeft;
    private final SparkMaxConfig extendConfigRight;
    private final SparkMaxConfig extendConfigLeft;
    private double tolerence = 0.4;

    private PIDController pid = new PIDController(Constants.IntakeConstants.p, Constants.IntakeConstants.i, Constants.IntakeConstants.d);
    private double goal = 0.25;    
    private RelativeEncoder encoder; 
                    
    public IntakeExtendo(){
        extendMotorRight = new SparkMax(Constants.IntakeConstants.extendMotorIDright, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        extendMotorLeft = new SparkMax(Constants.IntakeConstants.extendMotorIDleft, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        extendConfigRight = new SparkMaxConfig();
        extendConfigLeft = new SparkMaxConfig();
                    
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
            
    public void runExtendMotor(double voltage) {
        SmartDashboard.putNumber("intake extendo voltage", voltage);
        if (Math.abs(goal - getEncoderPosition()) < tolerence) {
            extendMotorLeft.setVoltage(0);
        } else {
        extendMotorLeft.setVoltage(voltage);
        }
    }

    public void runExtendMotorManual(double voltage) {
        SmartDashboard.putNumber("intake extendo voltage", voltage);
        
        extendMotorLeft.setVoltage(voltage);
        
    }
            
    public double getEncoderPosition(){
        return encoder.getPosition();
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
        if (getEncoderPosition() >= -5) {

            setGoal(getEncoderPosition() - 13);
        }
        else
        {
            setGoal(getEncoderPosition() + 15);
        }
    }

    public Command toggleIntakeCommand()
    {
        return this.runOnce(() -> toggleIntake());
    }

    public Command manualDown()
    {
        return this.runOnce(() -> runExtendMotorManual(-2));
    }

    public Command manualUp()
    {
        return this.runOnce(() -> runExtendMotorManual(10));
    }

    @Override
    public void periodic()
    {
        double pidOutput = pid.calculate(getEncoderPosition(), goal);
        runExtendMotor(pidOutput);
        //runExtendWithGravity();
        SmartDashboard.putNumber("intake extension pid output", pidOutput);
        SmartDashboard.putNumber("intake extension goal", goal);
        SmartDashboard.putNumber("intake extension current", getEncoderPosition());
        SmartDashboard.putNumber("intake encoder position", getEncoderPosition());
    }

    public Command resetTopCommand() {
        return this.runOnce(() -> encoder.setPosition(0)).andThen(this.runOnce(() -> setGoal(0)));
    }

    public Command resetBottomCommand() {
        return this.runOnce(() -> encoder.setPosition(Constants.IntakeConstants.extendedPosition + 0.25)).andThen(this.runOnce(() -> setGoal(Constants.IntakeConstants.extendedPosition + 0.25)));
    }

    public void resetToggle()
    {
        if (goal > -5) {

            encoder.setPosition(0);
            setGoal(0);
        }
        else 
        {
            encoder.setPosition(Constants.IntakeConstants.extendedPosition);
            setGoal(Constants.IntakeConstants.extendedPosition);
        }
    }

    public Command resetToggleCommand()
    {
        return this.runOnce(() -> resetToggle());
    }
}
