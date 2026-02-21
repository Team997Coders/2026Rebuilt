package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.HubLock;
import frc.robot.subsystems.vision.PAVController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.net.ContentHandler;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Hood extends SubsystemBase {
    
    public SparkMax hood = new SparkMax(Constants.ShooterConstants.hoodMotor, MotorType.kBrushed);
    public AbsoluteEncoder hoodRelativeEncoder = hood.getAbsoluteEncoder();

    public SparkMaxConfig hoodConfig = new SparkMaxConfig();
    public PIDController PIDHoodController = new PIDController(Constants.ShooterConstants.hoodPID.kp, Constants.ShooterConstants.hoodPID.ki, Constants.ShooterConstants.hoodPID.kd);

    private double goalAngle;
    private PAVController pav;
    private HubLock hubLock;

    public Hood(PAVController pav, HubLock hubLock) {
        this.pav = pav;
        this.hubLock = hubLock;
        hoodConfig.inverted(true);
        hood.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

      // setHoodAnglePos(25); //angle from horizontal to top of hood 

        goalAngle = 25;
    }

    @Override
    public void periodic() {
        setHoodMotorVoltage(PIDHoodController.calculate(getHoodAngle(), goalAngle));

        SmartDashboard.putNumber("Hood angle/pos", goalAngle);
        SmartDashboard.putNumber("hood angle", getHoodAngle());

        SmartDashboard.putNumber("hood pid outpud", PIDHoodController.calculate(getHoodAngle(), goalAngle));
    }

    //Hood
    public void setGoalAngle(double angle) {
        goalAngle = angle;
    }

    public double getHoodAngle () { //rad
        return hoodRelativeEncoder.getPosition()*360/Constants.ShooterConstants.hoodGearRatio;
    }

    public void setHoodMotorVoltage(double volts) {
        hood.setVoltage(volts);
    }

    public void moveHood(double angle) {
    
        if(angle > Constants.ShooterConstants.hoodBottomLimit && angle < Constants.ShooterConstants.hoodTopLimit) {
            setGoalAngle(angle);
        }
    }

    public void moveHoodUpManual() {
        if(goalAngle + 1 <= Constants.ShooterConstants.hoodTopLimit) {
       setGoalAngle(goalAngle+1);
        }
    }

    public void moveHoodDownManual() {
        if(goalAngle - 1 >= Constants.ShooterConstants.hoodBottomLimit) {
        setGoalAngle(goalAngle-1);
    } 
    }

    //lil commands
    //hood
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
