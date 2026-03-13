package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.PAVController;

public class PavShooter extends Command{

    private Shooter m_shooter;
    private HubLock m_hubLock;
    private PAVController m_pav;
    private Boolean finished = false;

    public PavShooter(Shooter shooter, HubLock hubLock, PAVController pav)
    {
        m_shooter = shooter;
        m_pav = pav;
        m_hubLock = hubLock;

        addRequirements(shooter);
    }

    @Override
    public void initialize()
    {
        finished = false;
    }

    @Override
    public void execute()
    {
        double distance = m_hubLock.getDistance();
        m_pav.update(distance);
        SmartDashboard.putNumber("distance from target", distance);
        m_shooter.moveFlywheel(m_pav.getVelocity() / Constants.ShooterConstants.flywheelRadius);
        SmartDashboard.putNumber("pav target velocity", m_pav.getVelocity());
    }

    @Override
    public void end(boolean interrupted)
    {
        m_shooter.setFlywheelVoltage(0);
    }

    @Override
    public boolean isFinished()
    {
        return finished;
    }

    public void finish()
    {
        finished = true;
    }

    public Command finishCommand()
    {
        return Commands.runOnce(() -> finish());
    }
}
