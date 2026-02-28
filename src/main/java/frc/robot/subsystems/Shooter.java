package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.vision.PAVController;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    
    //make sure to ramp up the wheels when running
    private TalonFX flywheel1 = new TalonFX(Constants.ShooterConstants.flywheelID);
    private TalonFX flywheel2 = new TalonFX(Constants.ShooterConstants.flywheel2ID);
    private TalonFXConfiguration flywheelConfig = new TalonFXConfiguration().withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    private PIDController shooterPID = new PIDController(0.055, 0, 0);
    private SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(0.1, 0.11/2/Math.PI, 0);

    public Shooter() {
        shooterPID.reset();
        flywheel2.setControl(new Follower(flywheel1.getDeviceID(), MotorAlignmentValue.Aligned)); 
        flywheelConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5;
        flywheelConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.5;
        flywheel1.setNeutralMode(NeutralModeValue.Coast);
        flywheel2.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Periodically updates the dashboard with the flywheel output, tangential velocity, and rotational velocity
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter output", flywheel1.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("shooter velocity rotational", getFlywheelRotVel());
        SmartDashboard.putNumber("shooter velocity tangential", getflywheelTanVel());
    }

    /**
     * Sets the voltage of the flywheels.
     * Limits the voltage to only positive numbers. 
     * (Dont run the flywheel backwards)
     */
    private double shooterVolts = 0.0;
    public void setFlywheelVoltage(double volts) {
        shooterVolts = volts;

        if (shooterVolts < 0) {
            shooterVolts = 0;
        }

       flywheel1.setVoltage(-shooterVolts);

       SmartDashboard.putNumber("volts", shooterVolts);
    }
    
    /**
     * @return the current rotational velocity of the flywheels in radians/second
     */
    public double getFlywheelRotVel () { 
        return -flywheel1.getVelocity().getValueAsDouble()*2*Math.PI; 
    }

    /**
     * @return the current tangential velocity of the flywheels in meters/second
     */
    public double getflywheelTanVel() { 
        return getFlywheelRotVel()*Constants.ShooterConstants.flywheelRadius; 
    }

    /**
     * Flywheel velocity is in RADIANS/SECOND
     * @param flywheelVel
     * Sets the rotational velocity (omega) of the shooter to requested speed using pid and ff
     */
    public void moveFlywheel(double flywheelVel) {
        double voltage = shooterPID.calculate(getFlywheelRotVel(), flywheelVel);
        double ff = shooterFF.calculate(flywheelVel);
       
        SmartDashboard.putNumber("shooter pid calculated volts", voltage);

        voltage += ff;

        SmartDashboard.putNumber("ff value", ff);
        SmartDashboard.putNumber("actual set voltage", voltage);
        SmartDashboard.putNumber("requested velocity", flywheelVel);

        setFlywheelVoltage(voltage);
    }

    /**
     * This method runs the flywheels with the TANGENTIAL velocity (in meters/second) specified on Smart Dashboard.
     * Change the "shooter velocity setpoint" on elastic to set specified velocity
     */
    public void flywheelWithDashboard()
    {
        moveFlywheel(SmartDashboard.getNumber("shooter velocity setpoint", 0.0) / Constants.ShooterConstants.flywheelRadius);
    }

    public Command runFlywheelVolt(double volts) {
        return this.runOnce(() -> setFlywheelVoltage(volts));
    }

    public Command moveFlywheelCommand(double velocity)
    {
        return this.run(() -> moveFlywheel(velocity));
    }

    public Command moveFlywheelDashboardCommand()
    {
        return this.run(() -> flywheelWithDashboard());
    }
}
