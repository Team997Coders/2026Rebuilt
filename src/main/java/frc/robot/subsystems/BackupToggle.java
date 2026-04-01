package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SubsystemCommands.HubLock;
import frc.robot.commands.SubsystemCommands.PavHood;
import frc.robot.commands.SubsystemCommands.PavShooter;

public class BackupToggle extends SubsystemBase{
    private Boolean state = false;
    private HubLock m_HubLock;
    private PavShooter m_PavShooter;
    private PavHood m_PavHood;
    private Hood hood;
    private Shooter shooter;
    private Command currentCommand;
    
    public BackupToggle(HubLock hubLock, PavShooter pavShooter, PavHood pavHood, Hood Hood, Shooter Shooter) {

        m_HubLock = hubLock;
        m_PavShooter = pavShooter;
        m_PavHood = pavHood;
        hood = Hood;
        shooter = Shooter;
        
    }
    

    public void periodic() {
        if (!state) {
            currentCommand = m_HubLock.alongWith(m_PavShooter).alongWith(m_PavHood);
        } else {
            currentCommand = shooter.moveFlywheelCommand(Constants.ShooterConstants.backupVoltage).alongWith(hood.hoodBackup());
        }

    }

   

    public void toggleShooter() {
       if (state) {
        state = false;
       } else {
        state = true;
       }
    }
    
     public Command shootingCommand() {
        return currentCommand;
    }  

    public Command toggleShooterCommand() {
        return this.runOnce(() -> toggleShooter());
    }


    
}
