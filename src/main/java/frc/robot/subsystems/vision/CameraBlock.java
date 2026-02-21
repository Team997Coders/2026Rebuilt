package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CameraBlock 
{
    ArrayList<Camera> cameraList;

    public CameraBlock(ArrayList<Camera> cameraList)
    {
        this.cameraList = cameraList;
    }

    public void update(SwerveDrivePoseEstimator poseEstimator)
    {
        SmartDashboard.putBoolean("update camera block", true);
        SmartDashboard.putNumber("number of cameras", this.cameraList.size());
        for (Camera camera: this.cameraList)
        {
            SmartDashboard.putNumber("camera #", camera.getClass().hashCode());
            //camera.update(poseEstimator);
            SmartDashboard.putNumber("testing camera other method", camera.getYawClump());
            SmartDashboard.putNumber("does this work for camera?", camera.update2(poseEstimator));
            SmartDashboard.putBoolean("made it to the other side of the method!", true);
        }
    }

    public Camera getObjectCamera()
    {
        return cameraList.get(0);
    }
}