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

public class ObjectCamera extends Camera
{
    public ObjectCamera(String cameraName, Transform3d robotToCamera)
    {
        super(cameraName, robotToCamera);
    }

    @Override
    public void update(SwerveDrivePoseEstimator poseEstimator)
    {
        this.results = this.camera.getAllUnreadResults();
    }

    public double getYaw()
    {
        SmartDashboard.putBoolean("results not empty", !results.isEmpty());
        if (!results.isEmpty())
        {
            PhotonPipelineResult pipelineResult = results.get(0);
            SmartDashboard.putBoolean("has targets", pipelineResult.hasTargets());
            if (pipelineResult.hasTargets())
            {
                PhotonTrackedTarget target = pipelineResult.getBestTarget();
                return target.yaw;
            }

        }
        return Double.MAX_VALUE;
    }


}