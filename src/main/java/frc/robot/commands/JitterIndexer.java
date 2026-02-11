package frc.robot.commands;

import frc.robot.subsystems.Indexer;

import java.util.TimerTask;
import java.time.Period;
import java.util.Timer;

import edu.wpi.first.wpilibj2.command.Command;

public class JitterIndexer extends Command {

    Timer timer;

    private Indexer m_indexer;

    public JitterIndexer(Indexer indexer) {
        timer = new Timer();

        this.m_indexer = indexer;

    }

    TimerTask task = new TimerTask() {

        @Override
        public void run() {
           if (m_indexer.getIndexVoltage() > 0) {
            //this is the speed it would run for the jitter, need to test
            m_indexer.spinIndexerMotor(-0.5);
           } else {
            m_indexer.spinIndexerMotor(0.5);
           }
        }
        
    };


    @Override
    public void initialize() {
        //timer.start();
        
    }

    @Override 
    public void execute() {
        timer.scheduleAtFixedRate(task, 0, 500);//the run period needs to be tested
    }
    
    @Override
    public boolean isFinished() {
        return false;   
    }


    @Override 
    public void end(boolean interrupted) {
        timer.cancel();
    }
}
