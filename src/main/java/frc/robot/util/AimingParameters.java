package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public record AimingParameters(
        Rotation2d robotAngle, double robotVelocityRadPerSec, Rotation2d hoodAngle, double shooterRPM) {}
