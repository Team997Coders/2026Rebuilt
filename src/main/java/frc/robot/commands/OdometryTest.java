package frc.robot.commands;


import static edu.wpi.first.units.Units.Revolution;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;
import swervelib.SwerveModule;

public class OdometryTest extends Command{

    private Drivebase drivebase;
    private double startPos;
    private double error;
    private PIDController pid = new PIDController(1, 0, 0);

  /** Creates a new Drive. */
  public OdometryTest(Drivebase drivebase, double startPos, double error) {
    this.drivebase = drivebase;
    this.startPos = startPos;
    this.error = error;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivebase);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startPos = drivebase.getPositions()[0].distanceMeters; 
    pid.reset();
    pid.setSetpoint(drivebase.getPositions()[0].distanceMeters - Constants.AutoDriveConstants.distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPos = drivebase.getPositions()[0].distanceMeters;
    double deltaPos = currentPos - startPos;
    double error = Constants.AutoDriveConstants.distance - deltaPos;
    SmartDashboard.putNumber("auto error", error);
    drivebase.defaultDrive(-pid.calculate(drivebase.getPositions()[0].distanceMeters), 0, 0);
    SmartDashboard.putNumber("pid output auto", pid.calculate(drivebase.getPositions()[0].distanceMeters));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}