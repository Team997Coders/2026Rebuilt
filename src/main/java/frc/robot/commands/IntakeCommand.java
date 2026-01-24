package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {

    private Intake intake;
    private double startPosition;
    private PIDController pid = new PIDController(Constants.IntakeConstants.p, Constants.IntakeConstants.i, Constants.IntakeConstants.d);
    private double goal;
    private boolean extend;
    

    public IntakeCommand(Intake intake, boolean extend) {
        this.intake = intake;
        this.extend = extend;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startPosition = intake.encoderPosition();
        pid.reset();
        
        // true for extend, false for retract
        if (extend){
            goal = Constants.IntakeConstants.goal;
        } else {
            goal = -Constants.IntakeConstants.goal;
        }
    
    }

    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double error = Constants.IntakeConstants.goal - intake.encoderPosition();
        SmartDashboard.putNumber("error", error);
        double currentPos = intake.encoderPosition() - startPosition;
        intake.runExtendMotor(pid.calculate(currentPos, goal));
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