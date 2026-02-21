package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NeoPixel extends SubsystemBase{

    private DoubleSupplier getMatchTime;
    private enum ledStates {Blue_Inactive, Red_Inactive, Blue_Active, Red_Active, Climb, Shoot, Hub_Lock};
    private ledStates state;

    public NeoPixel(DoubleSupplier getMatchTime)
    {
        this.getMatchTime = getMatchTime;
    }

    @Override
    public void periodic()
    {
        
    }

    public void setToDefault()
    {
        if (DriverStation.getAlliance().orElseThrow().equals(Alliance.Blue))
        {
            state = ledStates.Blue_Active;
        }
        else
        {
            state = ledStates.Red_Active;
        }
    }

    public void shooting(boolean shooting)
    {
        if (shooting)
        {
            state = ledStates.Shoot;
        }
        else if (state.equals(ledStates.Shoot))
        {
            setToDefault();
        }
    }

    public void climbing(boolean climbing)
    {
        if (climbing)
        {
            state = ledStates.Climb;
        }
        else if (state.equals(ledStates.Climb))
        {
            setToDefault();
        }
    }

    public Command shootingCommand(boolean shooting)
    {
        return this.runOnce(() -> this.shooting(shooting));
    }
}
