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
    
    
    public BackupToggle() {}

   

    public void toggleShooter() {
       if (state) {
        state = false;
       } else {
        state = true;
       }
    }

    public Boolean getState() {
        return state;
    }
    

    public Command toggleShooterCommand() {
        return this.runOnce(() -> toggleShooter());
    }


    
}
