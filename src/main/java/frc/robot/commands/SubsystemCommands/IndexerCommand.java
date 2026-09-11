package frc.robot.commands.SubsystemCommands;

import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.PAVController;

public class IndexerCommand extends Command{

    private Indexer m_indexer;
    private Boolean finished = false;
    private double IndexerSpeed = 0.67;
    private DoubleSupplier m_speed;

    public IndexerCommand(Indexer indexer, DoubleSupplier speed)
    {
        m_indexer = indexer;
        m_speed = speed;

        addRequirements(m_indexer);
    }

    @Override
    public void initialize()
    {
        finished = false;
    }

    @Override
    public void execute()
    {
        double speed = (m_speed.getAsDouble() - 0.5) * 2;
        //SmartDashboard.putNumber("trigger speed", speed);
        m_indexer.setIndexerMotor(speed);
    }

    @Override
    public void end(boolean interrupted)
    {
        m_indexer.setIndexerMotor(0);
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
