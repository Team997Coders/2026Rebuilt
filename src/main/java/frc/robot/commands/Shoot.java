package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.Camera;
import frc.robot.subsystems.vision.CameraShooter;
import frc.robot.Constants;
import frc.robot.commands.HubLock;

public class Shoot extends Command {
    private Shooter m_shooter;
    private Indexer m_indexer;
    private CameraShooter m_shootCamera;
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private Hood m_hood;



    public Shoot(Shooter shooter, Indexer indexer, CameraShooter shootCamera, Hood hood) {
        this.m_shooter = shooter;
        this.m_indexer = indexer;
        this.m_shootCamera = shootCamera;
        this.m_hood = hood;


        addRequirements(shooter, indexer, shootCamera);
    }

    @Override
    public void initialize() {

    }

    private double velocity;
    private final double deltaH = 1.83 - 0.6;
    public double getAngleGofX(double distance)
    {
      velocity = 10;
      double angle = 90 - (Math.atan(
            (Math.pow(velocity, 2) - 
            Math.sqrt(
                Math.pow(velocity, 4) +
                19.6 * (
                    (Math.pow(velocity, 2) * (0 - deltaH)) -
                    (4.9 * Math.pow(distance, 2))
                )
            )) 
            / (9.8 * distance))
                 )*180/Math.PI;

        return angle;
    }

    public double getAngleFofX(double distance)
    {
      velocity = 10;
      double angle = 90 - (Math.atan(
            (Math.pow(velocity, 2) - 
            Math.sqrt(
                Math.pow(velocity, 4) +
                19.6 * (
                    (Math.pow(velocity, 2) * (0 - deltaH)) -
                    (4.9 * Math.pow(distance, 2))
                )
            )) 
            / (9.8 * distance))
                 )*180/Math.PI;

        return angle;
    }

    @Override 
    public void execute() {

      //  m_shooter.moveRollerandFlywheel(-Constants.ShooterConstants.flywheelVoltage, Constants.ShooterConstants.rollerVoltage);
        Pose2d tag = aprilTagFieldLayout.getTagPose(10).orElseThrow().toPose2d();
        Pose2d goalPose = new Pose2d(tag.getX() + Units.inchesToMeters(47.0/2), tag.getY(), tag.getRotation());
        double angle = getAngleGofX(m_shootCamera.getDistanceFromTarget(goalPose));
        SmartDashboard.putNumber("shooter target angle", angle);

        if ((angle >= Constants.ShooterConstants.hoodBottomLimit) && (angle <= Constants.ShooterConstants.hoodTopLimit)){
            m_hood.setGoalAngle(angle);
        }

        
    }

    @Override
    public boolean isFinished() {
        return false;

    }
    

    @Override
    public void end(boolean interrupted) {
        m_shooter.setFlywheelVoltage(0);
    }
}


