// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import swervelib.SwerveModuleConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int gyroID = 46;

  public static class ShooterConstants {
    public static final int flywheelID = 15;
    public static final int flywheel2ID = 16;
    public static final int hoodMotor = 17;
    public static final int rollerMotor = 18;

    public static final double rollerVoltage = 3;
    public static final double rollerReverseVoltage = -2;

    public static final double flywheelRotationalVelocity = 2; 
    public static final double flywheelReverseVelocity = 0;

    public static final double flywheelVoltage = 4.2;

    public static final double hoodTopLimit = 75; //degrees 
    public static final double hoodBottomLimit = 43.3; 

    public static final double hoodGearRatio = 35*(300/28);

    public static final double kS = 0.1; // Volts to account for static friction
    public static final double kV = 0.12; // RPS per Volt, Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    public static final double kA = 0;  // Volt seconds per radian

    public static final int beamBreak = 10;
    public static final int magnet = 0;
    public static final int absoluteEncoder = 1;


    public static final class flywheelPID {
      public static final double kp = 0.1;
      public static final double ki = 0.00;
      public static final double kd = 0.00;
    }

    public static final class hoodPID {
      public static final double kp = 0.15;
      public static final double ki = 0.00;
      public static final double kd = 0.00;
    }

    public static final double flywheelGearRatio = 54/32; //54 is motor 32 is flywheel
    public static final double flywheelRadius = 0.0508; //in meters
  }

  public static final class IndexerConstants {
  public static final int indexerMotorID = 90; 
  public static final double typicalIndexOutputCurrent = 1; //this is the value indexer normaly uses when ball is not stuck
  public static final double speedToUnstick = -1; 
  public static final int disiredUnstickTime = 500;

  public final static double defaultVolts = 3;
  public final static double reverseVolts = -2;
  }

  public static final class DriveConstants {
    public static final double deadband = 0.02;
    public static final int currentLimit = 40;
    public static final double slewRate = 20; // lower number for higher center of mass

    public static final class SwervePID {
      public static final double p = 0.005;
      public static final double i = 0;
      public static final double d = 0;
    }

 

    public static final class SwerveModules {

      // Front Left Module
      public static final SwerveModuleConfig frontLeft = new SwerveModuleConfig(
          5,
          6,
          24,
          true,
          true,
          false,
          1,
          //.462
          0);

      // Front Right
      public static final SwerveModuleConfig frontRight= new SwerveModuleConfig(
          7,
          8,
          23,
          true,
          true,
          false,
          1,
          //0
          0);

      // Back Right
      public static final SwerveModuleConfig backRight = new SwerveModuleConfig(
          1,
          2,
          21,
          true,
          true,
          false,
          1,
          //.759
          0);

      // Back Left
      public static final SwerveModuleConfig backLeft = new SwerveModuleConfig(
          3,
          4,
          22,
          true,
          true,
          false,
          1,
          //.158 // 0 to 1
          0
      );
    }

    public static final class ModuleLocations {
      public static final double dist = Units.inchesToMeters(11.0);
      public static final double robotRaduius = Math.sqrt(2 * Math.pow(dist, 2));
      public static final Translation2d frontLeft = new Translation2d(dist, dist);
      public static final Translation2d frontRight = new Translation2d(dist, -dist);
      public static final Translation2d backLeft = new Translation2d(-dist, dist);
      public static final Translation2d backRight = new Translation2d(-dist, -dist);
    }
  }
  public static final class AutoConstants {
    public static final class XPID {
      public static final double p = 1.5;
      public static final double i = 0;
      public static final double d = 0;
    }

    public static final class YPID {
      public static final double p = 1.5;
      public static final double i = 0;
      public static final double d = 0;
    }

    public static final class RPID {
      public static final double p = 0.0015;
      public static final double i = 0;
      public static final double d = 0.0002;
    }

    public static final int medianFilter = 5;
  }

  public static final class PathPlannerConstants {
    public static final class TranslationPID {
      public static final double p = 5;
      public static final double i = 0;
      public static final double d = 0;
    }

    public static final class RotationPID {
      public static final double p = 6;
      public static final double i = 0;
      public static final double d = 0;
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class CANdleConstants {
    public static final int id = 50;
    public static final int ledCount = 50;
  }

   public static final class IntakeConstants {
    public static final int spinMotorID = 91;
    public static final int extendMotorID = 92;

    public static final double spinVoltage = 0;

    public static final int p = 0;
    public static final int i = 0;
    public static final int d = 0;
    public static final double goal = 0;
  }



public static double airTime = 1;
}
