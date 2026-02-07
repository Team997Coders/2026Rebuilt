package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Indexer;



public class Unstick extends Command {

    private Indexer m_indexer;

    public Unstick (Indexer indexer) {
        this.m_indexer = indexer;
        addRequirements(m_indexer);
    }

    int counter = 0;

    @Override
    public void initialize() {
        
        
    }

    @Override
    public void execute() {
        counter++;
        if (counter == Constants.IndexerConstants.disiredUnstickTime){ 
            end(true);
        }
    }
    
    @Override
    public boolean isFinished() {
        return false;

}

    @Override
    public void end(boolean interrupted) {
        m_indexer.startIdexCommand();
    }
}
