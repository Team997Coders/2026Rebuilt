package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Climber extends SubsystemBase {

    String level;
    int indexlevel;
    double groundToLevel1 = 12; 
    double otherLevelDifferences = 18; // modify level differences, based of inch's betweens climb levels

    private final SparkMax leftClimber = new SparkMax(Constants.ClimberConstants.leftClimberID, MotorType.kBrushless);

    private final SparkMaxConfig leftConfig =  new SparkMaxConfig();

    private final RelativeEncoder leftEncoder = leftClimber.getEncoder();

    private final DigitalInput limit = new DigitalInput(Constants.ClimberConstants.limitChannel);

    private final PIDController climbPid = new PIDController(Constants.ClimberConstants.kP, Constants.ClimberConstants.kI, Constants.ClimberConstants.kD);

    public Climber () {
        leftConfig.inverted(Constants.ClimberConstants.leftInverted);

        leftClimber.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    @Override
    public void periodic() {
        limitSwitchReset();
    }

       private boolean hasReset = false; 
    private boolean stopClimberVolts = false;

    public void setClimberVolts(double volts) {
        if (stopClimberVolts) {
            leftClimber.setVoltage(0);
            return;
        }
        leftClimber.setVoltage(volts);
    }

    public void goToPos(double encoderPos) {
        double output = climbPid.calculate(getPosition(), encoderPos);

    
        if (limit.get() && output < 0) {
            setClimberVolts(0);
            return;
        }

        if (encoderPos > getPosition() && this.indexlevel >= 1) {
            setClimberVolts(0);
            return;
        }

        setClimberVolts(output);
    }

    public void limitSwitchReset() {
        if (limit.get() && !hasReset) {
            resetEncoder();
            hasReset = true;
        }
        if (!limit.get()) {
            hasReset = false; 
        }
    }

    public void stopClimber() {
        stopClimberVolts = true;
        leftClimber.setVoltage(0);
    }

    public void resetStopClimber() {
        stopClimberVolts = false;
    }

    public double getPosition() {
        return leftEncoder.getPosition();
    }

    public void resetEncoder() {
        leftEncoder.setPosition(0);
    }

    public void climberLevel(double groundToLevel1) {
        this.indexlevel = (int) Math.ceil(getPosition() / groundToLevel1);
        this.level = String.valueOf(this.indexlevel);
    }

    public String getLevel() {
        return this.level;
    }
}    