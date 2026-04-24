package frc.robot.commands;

import static edu.wpi.first.units.Units.NewtonMeter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;

public class objectDetectionPathfinding extends Command{
    
    // Since we are using a holonomic drivetrain, the rotation component of this pose
// represents the goal holonomic rotation
    private Drivebase m_drivebase;

    public objectDetectionPathfinding(Drivebase drivebase){
        m_drivebase = drivebase;
    }

    private Rotation2d currentRotation = m_drivebase.getPose().getRotation();
    

    private Pose2d targetPose = new Pose2d(m_drivebase.getPose().getX() + 
        Constants.ObjectDetectionConstants.desiredDistance*Math.cos(currentRotation.getRadians()), 
        m_drivebase.getPose().getY() + Constants.ObjectDetectionConstants.desiredDistance*Math.sin(currentRotation.getRadians()), 
        currentRotation);
  
    // Create the constraints to use while pathfinding
    private PathConstraints constraints = new PathConstraints(
        3.0, 5.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    private Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0

    
    );

    private Command autonomousCommand = new PathPlannerAuto("DepotTrench");

}
