package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.hubLock;
import frc.robot.subsystems.vision.PAVController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    public DigitalInput hoodSwitch;
    public boolean hoodZeroed = false;

    public SparkMax hood = new SparkMax(Constants.ShooterConstants.hoodMotor, MotorType.kBrushed);
    public RelativeEncoder hoodRelativeEncoder = hood.getEncoder();

    public SparkMaxConfig hoodConfig = new SparkMaxConfig();

    public PIDController PIDHoodController = new PIDController(Constants.ShooterConstants.hoodPID.kp,
            Constants.ShooterConstants.hoodPID.ki, Constants.ShooterConstants.hoodPID.kd);

    private double goalAngle;
    private PAVController pav;
    private hubLock hubLock;

    public Hood(PAVController pav, hubLock hubLock) {
        this.pav = pav;
        this.hubLock = hubLock;
        // hoodConfig.inverted(true);
        hood.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // setHoodAnglePos(25); //angle from horizontal to top of hood

        hoodRelativeEncoder.setPosition(25.0 * Constants.ShooterConstants.hoodGearRatio / 360);
        goalAngle = 25;
    }

    @Override
    public void periodic() {
        setHoodMotorVoltage(PIDHoodController.calculate(getHoodAngle(), goalAngle));
        SmartDashboard.putNumber("Hood angle/pos", goalAngle);
        SmartDashboard.putNumber("hood angle", getHoodAngle());
        SmartDashboard.putNumber("hood pid output", PIDHoodController.calculate(getHoodAngle(), goalAngle));
        SmartDashboard.putBoolean("Hood Zeroed?", hoodZeroed);
    }

    // Hood
    public void setGoalAngle(double angle) {
        goalAngle = angle;
    }

    public double getHoodAngle() { // degrees
        return hoodRelativeEncoder.getPosition() * 360 / Constants.ShooterConstants.hoodGearRatio;
    }

    public void setHoodMotorVoltage(double volts) {
        hood.setVoltage(-volts);
    }

    public void moveHood(double angle) {
        if (angle > Constants.ShooterConstants.hoodBottomLimit && angle < Constants.ShooterConstants.hoodTopLimit) {
            setGoalAngle(angle);
        }
    }

    public void moveHoodUpManual() {
        if (goalAngle + 1 <= Constants.ShooterConstants.hoodTopLimit) {
            setGoalAngle(goalAngle + 1);
        }
    }

    public void moveHoodDownManual() {
        if (goalAngle - 1 >= Constants.ShooterConstants.hoodBottomLimit) {
            setGoalAngle(goalAngle - 1);
        }
    }

    // lil commands
    // hood
    public Command hoodUp() {
        return this.run(() -> moveHoodUpManual());
    }

    public Command hoodDown() {
        return this.run(() -> moveHoodDownManual());
    }

    public void PAVcontrollerAngle() {
        pav.update(hubLock.getDistanceFromTarget(hubLock.getGoalPose()));
        this.setGoalAngle(pav.getAngle());

    }

    public Command PAVcommand() {
        return this.run(() -> PAVcontrollerAngle());
    }

}
