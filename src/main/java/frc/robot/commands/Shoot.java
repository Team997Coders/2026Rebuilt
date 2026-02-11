package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import frc.robot.commands.HubLock;

public class Shoot extends Command {
    private Shooter m_shooter;
    private Indexer m_indexer;
    private HubLock m_hubLock;

    private Trigger beamBreak = m_shooter.beamBreakTrigger;

    public Shoot(Shooter shooter, Indexer indexer, HubLock hubLock) {
        this.m_shooter = shooter;
        this.m_indexer = indexer;
        this.m_hubLock = hubLock;

        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {

    }

    private double velocity;
    private final double deltaH = 0.0;
    public double getAngle(double distance)
    {
        velocity = m_shooter.getflywheelTanVel();
        return 90 - Math.atan(
            (Math.pow(velocity, 2) - 
            Math.sqrt(
                Math.pow(velocity, 4) +
                19.6 * (
                    Math.pow(velocity, 2) * deltaH -
                    4.9 * Math.pow(distance, 2)
                )
            )) 
            / (9.8 * distance)
        );
    }

    @Override 
    public void execute() {
        m_shooter.setRollerVoltage(Constants.ShooterConstants.rollerVoltage);
        m_shooter.runFlywheelVolt(Constants.ShooterConstants.flywheelVoltage);
        double angle = getAngle(m_hubLock.getDistance());
        SmartDashboard.putNumber("shooter target angle", angle);
        m_shooter.setGoalAngle(angle);
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    

    @Override
    public void end(boolean interrupted) {
    }
}
