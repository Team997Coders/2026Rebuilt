package frc.robot.subsystems;

import static edu.wpi.first.units.Units.NewtonMeter;

import java.security.AlgorithmConstraints;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.clumpLock;
import frc.robot.subsystems.Drivebase;

public class objectDetectionPathfinding extends SubsystemBase{
    

    private Drivebase m_drivebase;
    private clumpLock m_clumpLock;

    public objectDetectionPathfinding(Drivebase drivebase, clumpLock clumpLock){
        m_drivebase = drivebase;
        m_clumpLock = clumpLock;
    }

    private Rotation2d currentRotation = m_drivebase.getPose().getRotation();

    /* Set target pose for adaptivee path: goes a desired distance in the direction of the largest clump,
     as determined by objectLock */
    private Pose2d targetPose = new Pose2d(m_drivebase.getPose().getX() + 
        Constants.ObjectDetectionConstants.desiredDistance*Math.cos(currentRotation.getRadians()), 
        m_drivebase.getPose().getY() + Constants.ObjectDetectionConstants.desiredDistance*Math.sin(currentRotation.getRadians()), 
        currentRotation);

    // Pose the robot travels to after object detection
    private Pose2d target2 = new Pose2d(Constants.ObjectDetectionConstants.desiredX, 
        Constants.ObjectDetectionConstants.desiredY,
        Constants.ObjectDetectionConstants.desiredRotation);
  
    // Create the constraints to use while pathfinding
    private PathConstraints constraints = new PathConstraints(
        3.0, 5.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Build an auto with determined pose
    private Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0
    );

    // Pathfind to desired static position 
    private Command pathfindBack = AutoBuilder.pathfindToPose(
        target2,
        constraints,
        0.0
    );

    // Paths made in Pathplanner to start and end sequence
    private Command autonomousCommand = new PathPlannerAuto(Constants.ObjectDetectionConstants.startAuto);
    private Command auto2 = new PathPlannerAuto(Constants.ObjectDetectionConstants.endAuto);

    // Put it all together into an auto that can be chosen in AutoChooser
    public Command coolCommand() {
        return new SequentialCommandGroup(
            autonomousCommand,
            new WaitCommand(.5),
            m_clumpLock,
            new WaitCommand(.5),
            pathfindingCommand,
            new WaitCommand(.5),
            pathfindBack,
            new WaitCommand(.5),
            auto2
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }        
    
}