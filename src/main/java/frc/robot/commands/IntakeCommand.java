package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {

    private Intake intake;
    private double startPosition;
    private PIDController pid = new PIDController(Constants.IntakeConstants.p, Constants.IntakeConstants.i, Constants.IntakeConstants.d);
    private 

    public IntakeCommand(Intake intake, double startPosition) {
        this.intake = intake;
        this.startPosition = startPosition;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startPosition = Intake.encoderPosition();
        pid.reset();
    }

    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
            return false;
        
    }
}