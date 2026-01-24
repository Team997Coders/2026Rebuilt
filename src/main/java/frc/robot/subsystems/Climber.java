package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
\


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private final SparkMax leftClimber = new SparkMax(Constants.ClimberConstants.leftClimberID, MotorType.kBrushless);
    private final SparkMax rightClimber = new SparkMax(Constants.ClimberConstants.rightClimberID, MotorType.kBrushless);

    private final SparkMax tilt = new SparkMax(Constants.ClimberConstants.tiltID, MotorType.kBrushless);

    private final SparkMaxConfig leftConfig =  new SparkMaxConfig();
    private final SparkMaxConfig rightConfig = new SparkMaxConfig();

    private final SparkMaxConfig tiltConfig = new SparkMaxConfig();

    private final RelativeEncoder leftEncoder = leftClimber.getEncoder();

    private final DigitalInput limit = new DigitalInput(Constants.ClimberConstants.limitChannel);

    private final PIDController climbPid = new PIDController(Constants.ClimberConstants.kP, Constants.ClimberConstants.kI, Constants.ClimberConstants.kD);

    public Climber () {
        leftConfig.inverted(Constants.ClimberConstants.leftInverted);
        rightConfig.inverted(Constants.ClimberConstants.rightInverted);

        tiltConfig.inverted(Constants.ClimberConstants.tiltInverted);

        rightConfig.follow(Constants.ClimberConstants.leftClimberID);

        leftClimber.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rightClimber.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        tilt.configure(tiltConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic() {

    }


    public enum ClimberHeight {
        DOWN("DOWN", Constants.ClimberConstants.Heights.DOWN, 0),
        GRAB("LOWER GRAB", Constants.ClimberConstants.Heights.GRAB, 1),
        RAISE("LOWER RAISE", Constants.ClimberConstants.Heights.RAISE, 2);

        String name;
        double encoderPos;
        int index;
    
        ClimberHeight(String name, double encoderPos, int index) {
            this.name = name;
            this.encoderPos = encoderPos;
            this.index = index;

        }
    }


    public void setClimberVolts(double volts) {
        leftClimber.setVoltage(volts);
    }

    public void setTiltVolts(double volts){
        tilt.setVoltage(volts);
    }

    public void resetEncoder() {
        leftEncoder.setPosition(0);
    }

    public double getPosition() {
        return leftEncoder.getPosition();
    }

    public void goToPos(double encoderPos) {
        setClimberVolts(climbPid.calculate(getPosition(), encoderPos));
    }


}
