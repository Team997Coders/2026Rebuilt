package frc.robot.subsystems.vision;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class Camera
{
    protected PhotonCamera camera;
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    private PhotonPoseEstimator photonPoseEstimator;

    protected List<PhotonPipelineResult> results;

    public Camera(String cameraName, Transform3d robotToCamera)
    {
        this.camera = new PhotonCamera(cameraName);

        this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, robotToCamera);

        this.results = null;
    }

    public void update(SwerveDrivePoseEstimator poseEstimator)
    {
        SmartDashboard.putBoolean("camera update", true);
        results = this.camera.getAllUnreadResults();
        if (!this.results.isEmpty())
        {
            SmartDashboard.putBoolean("results is not empty", true);
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                SmartDashboard.putBoolean("results has targets", true);
                Optional<EstimatedRobotPose> optionalPose = this.photonPoseEstimator.estimateAverageBestTargetsPose(result);
                if (optionalPose.isPresent())
                {
                    SmartDashboard.putBoolean("results pose present", true);
                    EstimatedRobotPose estimatedRobotPose = optionalPose.orElseThrow();
                    poseEstimator.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), result.getTimestampSeconds());
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

    public Translation2d robot_to_tag(Drivebase drivebase)
    {
        if (!this.results.isEmpty())
        {
            PhotonTrackedTarget target = results.get(0).getBestTarget();
            if (target != null)
            {
                Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
                if (tagPose.isPresent())
                {
                    return PhotonUtils.estimateCameraToTargetTranslation(PhotonUtils.getDistanceToPose(drivebase.getPose(), tagPose.orElseThrow().toPose2d()), Rotation2d.fromDegrees(-target.getYaw()));
                }
            }
        }
        return new Translation2d();
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
}