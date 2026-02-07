package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Shoot extends Command {
    private Shooter m_shooter;
    private Indexer m_indexer;

    private Trigger beamBreak = m_shooter.beamBreakTrigger;

    public Shoot(Shooter shooter, Indexer indexer) {
        this.m_shooter = shooter;
        this.m_indexer = indexer;

        addRequirements(shooter, indexer);
    }

    @Override
    public void initialize() {

    }

    @Override 
    public void execute() {
        beamBreak.onFalse(new ParallelCommandGroup(m_indexer.startIndexer(), m_shooter.moveRoller(), m_shooter.runFlywheel()));
        beamBreak.onTrue(new ParallelCommandGroup(m_indexer.stopIndexer(), m_shooter.stopRoller(), m_shooter.runFlywheel()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    

    @Override
    public void end(boolean interrupted) {
    }
}
