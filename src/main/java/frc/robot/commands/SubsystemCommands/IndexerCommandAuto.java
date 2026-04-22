package frc.robot.commands.SubsystemCommands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal.SignalMeasurement;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.PAVController;

public class IndexerCommandAuto extends Command{

    private Indexer m_indexer;
    private Boolean finished = false;

    public IndexerCommandAuto(Indexer indexer)
    {
        m_indexer = indexer;

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
        //SmartDashboard.putNumber("trigger speed", speed);
        m_indexer.setIndexerMotor(0.67);
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
