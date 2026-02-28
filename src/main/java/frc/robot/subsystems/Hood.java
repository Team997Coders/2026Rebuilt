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
    public DigitalInput hoodSwitch;

    public SparkMax hood = new SparkMax(Constants.ShooterConstants.hoodMotor, MotorType.kBrushed);
    public RelativeEncoder hoodRelativeEncoder = hood.getEncoder();

    public SparkMaxConfig hoodConfig = new SparkMaxConfig();

    public PIDController PIDHoodController = new PIDController(Constants.ShooterConstants.hoodPID.kp, Constants.ShooterConstants.hoodPID.ki, Constants.ShooterConstants.hoodPID.kd);

    private double goalAngle;
    private PAVController pav;
    private HubLock hubLock;
    private double initializeCounter;

    public Hood(PAVController pav, HubLock hubLock) {
        this.pav = pav;
        this.hubLock = hubLock;
        //hoodConfig.inverted(true);
        hoodConfig.smartCurrentLimit(Constants.ShooterConstants.normalCurrentLimit);
        hood.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        //setHoodAnglePos(25); //angle from horizontal to top of hood 

        hoodRelativeEncoder.setPosition(25.0*Constants.ShooterConstants.hoodGearRatio/360);
        goalAngle = 25;
        initializeCounter = 0;
        hoodSwitch = new DigitalInput(Constants.ShooterConstants.beamBreak);
    }

    @Override
    public void periodic() {
        if (initializeCounter == 0) {
            while (!this.checkSwitch()) {
            hoodConfig.smartCurrentLimit(Constants.ShooterConstants.lowCurrentLimit);
            hood.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            this.moveHoodDownManual();
        }
        }

        setHoodMotorVoltage(PIDHoodController.calculate(getHoodAngle(), goalAngle));
        SmartDashboard.putNumber("Hood angle/pos", goalAngle);
        SmartDashboard.putNumber("hood angle", getHoodAngle());

        SmartDashboard.putNumber("hood pid outpud", PIDHoodController.calculate(getHoodAngle(), goalAngle));
    }

    //Hood
    public void setGoalAngle(double angle) {
        goalAngle = angle;
    }

    public double getHoodAngle () { //degrees
        return hoodRelativeEncoder.getPosition()*360/Constants.ShooterConstants.hoodGearRatio;
    }

    public void setHoodMotorVoltage(double volts) {
        hood.setVoltage(-volts);
    }

    public void moveHood(double angle) {
    
        if(angle > Constants.ShooterConstants.hoodBottomLimit && angle < Constants.ShooterConstants.hoodTopLimit) {
            setGoalAngle(angle);
        }
    }

    public void moveHoodUpManual() {
        initializeCounter += 1;
        if(goalAngle + 1 <= Constants.ShooterConstants.hoodTopLimit) {
       setGoalAngle(goalAngle+1);
        }
    }

    public void moveHoodDownManual() {
        if(goalAngle - 1 >= Constants.ShooterConstants.hoodBottomLimit) {
        setGoalAngle(goalAngle-1);
    } 
    }

    public void resetEncoder() {
        initializeCounter += 1;
        hoodRelativeEncoder.setPosition(0);
        hood.setVoltage(0);
        hoodConfig.smartCurrentLimit(Constants.ShooterConstants.normalCurrentLimit);
        hood.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    }

    public boolean checkSwitch() {
        return hoodSwitch.get();
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

    public Command reset() {
        return this.runOnce(() -> resetEncoder());
    }
}
