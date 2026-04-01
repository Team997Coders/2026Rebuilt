package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.BackupToggle;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.PAVController;

public class PavHood extends Command{

    private Hood m_hood;
    private HubLock m_hubLock;
    private PAVController m_pav;
    private BackupToggle m_BackupToggle;
    private Boolean finished = false;

    public PavHood(Hood hood, HubLock hubLock, PAVController pav, BackupToggle backupToggle)
    {
        m_hood = hood;
        m_pav = pav;
        m_hubLock = hubLock;
        m_BackupToggle = backupToggle;

        addRequirements(hood);
    }

    @Override
    public void initialize()
    {
        finished = false;
    }

    @Override
    public void execute()
    {
        if (!m_BackupToggle.getState()) {
            m_pav.update(m_hubLock.getDistance());
            m_hood.setGoalAngle(m_pav.getAngle());
        } else {
            m_hood.setGoalAngle(25);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
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
