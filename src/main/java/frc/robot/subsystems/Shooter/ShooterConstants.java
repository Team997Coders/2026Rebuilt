package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

public class ShooterConstants {
    //TODO: correct transform 3d and 2d for shooter
    public static final Transform3d ROBOT_TO_SHOOTER_3D = new Transform3d(3, 2, 0.0, new Rotation3d());
    public static final Transform2d ROBOT_TO_SHOOTER_2D = new Transform2d(3, 2, new Rotation2d());
}