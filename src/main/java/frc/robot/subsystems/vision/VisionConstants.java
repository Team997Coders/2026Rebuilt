// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  public static String pi5shootCamName = "tangerine";
  public static String pi5backCamName = "mango";
  public static String Opi5shootCamName = "pineapple";
  public static String Opi5sideCamName = "backberry";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  //TODO: check camera transorms cause they are very scuffed and different in lots of places
  public static Transform3d pi5shootCamTransform =
      new Transform3d(Units.inchesToMeters(-12), Units.inchesToMeters(13), Units.inchesToMeters(19), new Rotation3d(0.0, Units.degreesToRadians(10), Math.PI/2));
  public static Transform3d pi5backCamTransform =
      new Transform3d(Units.inchesToMeters(-12), Units.inchesToMeters(-13), Units.inchesToMeters(8), new Rotation3d(0.0, Units.degreesToRadians(10), -Math.PI/2));
  public static Transform3d Opi5shootCamTransform =
      new Transform3d(Units.inchesToMeters(-11.5), Units.inchesToMeters(13.25), Units.inchesToMeters(8), new Rotation3d(0.0, Units.degreesToRadians(10), Math.PI/2));
  public static Transform3d Opi5sideCamTransform =
      new Transform3d(Units.inchesToMeters(-12), Units.inchesToMeters(13), Units.inchesToMeters(8), new Rotation3d(0.0, Units.degreesToRadians(10), Math.PI));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  public static final double XY_STD_DEV_COEFF = 0.25;
  public static final double ROT_STD_DEV_COEFF = 0.25;

  public static final double MIN_ACCEPTED_NUM_TAGS = 1;
  public static final double MAX_AMBIGUITY = 0.2;
  public static final double MAX_OUTSIDE_OF_FIELD_TOLERANCE = 0.1;
  public static final double MAX_ROBOT_Z_TOLERANCE = 0.5;
}
