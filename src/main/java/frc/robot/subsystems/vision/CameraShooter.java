package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class CameraShooter extends SubsystemBase
{
    protected PhotonCamera camera;
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private PhotonPoseEstimator photonPoseEstimator;
    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private Field2d field = new Field2d();
    protected List<PhotonPipelineResult> results;

    private SwerveModulePosition[] modules = {new SwerveModulePosition(0, new Rotation2d()), new SwerveModulePosition(0, new Rotation2d()), new SwerveModulePosition(0, new Rotation2d()), new SwerveModulePosition(0, new Rotation2d())};


    public CameraShooter(String cameraName, Transform3d robotToCamera)
    {
        this.camera = new PhotonCamera(cameraName);
        
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(new SwerveDriveKinematics(new Translation2d(), new Translation2d(), new Translation2d(), new Translation2d()), new Rotation2d(), modules, new Pose2d());
        this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, robotToCamera);

        this.results = null;
    }

    public void update(SwerveDrivePoseEstimator poseEstimator)
    {
        results = this.camera.getAllUnreadResults();
        SmartDashboard.putBoolean("results not empty", !this.results.isEmpty());
        if (!this.results.isEmpty())
        {
            var result = results.get(results.size() - 1);
            SmartDashboard.putBoolean("has targets", result.hasTargets());
            if (result.hasTargets()) {
                Optional<EstimatedRobotPose> optionalPose = this.photonPoseEstimator.estimateAverageBestTargetsPose(result);
                SmartDashboard.putBoolean("optional pose present", optionalPose.isPresent());
                if (optionalPose.isPresent())
                {
                    EstimatedRobotPose estimatedRobotPose = optionalPose.orElseThrow();
                    poseEstimator.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), result.getTimestampSeconds());
                    SmartDashboard.putData("Field", field);
                    field.setRobotPose(poseEstimator.getEstimatedPosition());
                    poseEstimator.update(new Rotation2d(), modules);
                }
            }
        } 
    }

    public List<PhotonPipelineResult> getResults()
    {
        return this.camera.getAllUnreadResults();
    }

    public boolean hasTarget()
    {
        if (!this.results.isEmpty())
        {
            if (results.get(0).getBestTarget() != null)
            {
                return true;
            }
        }
        return false;
    }

   

    public Pose2d get_tag_pose2d()
    {
        if (!this.results.isEmpty())
        {
            PhotonTrackedTarget target = results.get(0).getBestTarget();
            if (target != null)
            {
                Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
                if (tagPose.isPresent())
                {
                    return tagPose.orElseThrow().toPose2d();
                }
            }
        }
        return new Pose2d();
    }

    public double getYawClump() {
        return Double.MAX_VALUE;
    }

public final double  getDistanceFromTarget(Pose2d goal)
  {
    Pose2d pose = swerveDrivePoseEstimator.getEstimatedPosition();
    return Math.sqrt(Math.pow(goal.getX() - pose.getX() , 2) + Math.pow(goal.getY() - pose.getY(), 2));
    
  }
  

  @Override
    public void periodic(){
        this.update(swerveDrivePoseEstimator);
        field.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
        Pose2d tag = aprilTagFieldLayout.getTagPose(10).orElseThrow().toPose2d();
        Pose2d goalPose = new Pose2d(tag.getX() + Units.inchesToMeters(47.0/2), tag.getY(), tag.getRotation());
        SmartDashboard.putNumber("distance from hub", this.getDistanceFromTarget(goalPose));
        
    }
}