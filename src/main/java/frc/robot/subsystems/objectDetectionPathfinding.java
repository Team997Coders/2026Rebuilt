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
import frc.robot.subsystems.Drivebase;

public class objectDetectionPathfinding extends SubsystemBase{
    
    // Since we are using a holonomic drivetrain, the rotation component of this pose
// represents the goal holonomic rotation
    private Drivebase m_drivebase;

    public objectDetectionPathfinding(Drivebase drivebase){
        m_drivebase = drivebase;
     }

          private   Rotation2d currentRotation = m_drivebase.getPose().getRotation();

        private Pose2d targetPose = new Pose2d(m_drivebase.getPose().getX() + 
            Constants.ObjectDetectionConstants.desiredDistance*Math.cos(currentRotation.getRadians()), 
            m_drivebase.getPose().getY() + Constants.ObjectDetectionConstants.desiredDistance*Math.sin(currentRotation.getRadians()), 
            currentRotation);

        private Pose2d target2 = new Pose2d(Constants.ObjectDetectionConstants.desiredX, Constants.ObjectDetectionConstants.desiredY,
            Constants.ObjectDetectionConstants.desiredRotation);
  
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

        private Command pathfindBack = AutoBuilder.pathfindToPose(
            target2,
            constraints,
            0.0
        );
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
        private Command autonomousCommand = new PathPlannerAuto(Constants.ObjectDetectionConstants.startAuto);
        private Command auto2 = new PathPlannerAuto(Constants.ObjectDetectionConstants.endAuto);

         private Command group3 = new SequentialCommandGroup(
            autonomousCommand,
            new WaitCommand(.5),
            pathfindingCommand,
            new WaitCommand(.5),
            pathfindBack,
            new WaitCommand(.5),
            auto2
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    
    
   
    public Command coolCommand() {
        return new SequentialCommandGroup(
            autonomousCommand,
            new WaitCommand(.5),
            pathfindingCommand,
            new WaitCommand(.5),
            pathfindBack,
            new WaitCommand(.5),
            auto2
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        }
        
    
    }