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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private final SparkMax climber = new SparkMax(Constants.ClimberConstants.climberID, MotorType.kBrushless);

    private final SparkMaxConfig config = new SparkMaxConfig();

    private final RelativeEncoder encoder = climber.getEncoder();

    private final DigitalInput limit = new DigitalInput(Constants.ClimberConstants.limitChannel);

    private final PIDController climbPid = new PIDController(Constants.ClimberConstants.kP, Constants.ClimberConstants.kI, Constants.ClimberConstants.kD);

    public Climber () {
        config.inverted(Constants.ClimberConstants.inverted);

        climber.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        encoder.setPosition(0);
    }

    double goalPos;
    @Override
    public void periodic() {
        // if(limit.get()) {
        //     resetEncoder();
        // }

        SmartDashboard.putNumber("climber pos", encoder.getPosition());
    
        //setClimberVolts(climbPid.calculate(getPosition(), goalPos));

    }



    public void setClimberVolts(double volts) {
        climber.setVoltage(volts);
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public void goToPos(double encoderPos) {
        goalPos = encoderPos;
    }

    public void manualUp() {
        goalPos+=1;
    }

    public void manualDown() {
        goalPos-=1;
    }

    public Command climberVoltsCommand(double volts)
    {
        return this.runOnce(() -> setClimberVolts(volts));
    }


    public Command raise() {
        return this.runOnce(() -> goToPos(Constants.ClimberConstants.raisedPos));
    }

    public Command lower() {
        return this.runOnce(() -> goToPos(Constants.ClimberConstants.loweredPos));
    }


}
