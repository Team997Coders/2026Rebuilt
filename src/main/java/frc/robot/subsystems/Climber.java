package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

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

    public Climber () {
        leftConfig.inverted(Constants.ClimberConstants.leftInverted);
        rightConfig.inverted(Constants.ClimberConstants.rightInverted);

        tiltConfig.inverted(Constants.ClimberConstants.tiltInverted);

        rightConfig.follow(Constants.ClimberConstants.leftClimberID);

        leftClimber.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rightClimber.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        tilt.configure(tiltConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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
}
