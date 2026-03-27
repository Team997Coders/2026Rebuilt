package frc.robot.commands.SubsystemCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.IntakeSpinny;

public class IntakeFuel extends Command{

    private IntakeSpinny m_intake;
    private Boolean finished = false;

    public IntakeFuel(IntakeSpinny intake)
    {
        m_intake = intake;

        addRequirements(m_intake);
    }

    @Override
    public void initialize()
    {
        finished = false;
    }

    @Override
    public void execute()
    {
        m_intake.output();
    }

    @Override
    public void end(boolean interrupted)
    {
        m_intake.spin(0);
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
