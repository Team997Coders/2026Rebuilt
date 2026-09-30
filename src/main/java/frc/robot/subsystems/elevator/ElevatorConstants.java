package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;

public class ElevatorConstants {
    public static final double elevatorSimP = 0.5;
    public static final double elevatorSimI = 0.0;
    public static final double elevatorSimD = 0.0;
    public static final double elevatorP = 0.5;
    public static final double elevatorI = 0.0;
    public static final double elevatorD = 0.0;

    public static final DCMotor frontGearbox = DCMotor.getNEO(1);
    public static final DCMotor backGearbox = DCMotor.getNEO(1);

    public static final double frontMotorReduction = 6.75;
    public static final double backMotorReduction = 6.75;

    //Making up numbers :)
    public static final double maxExtensionMeters = 0.7;
    public static final double maxExtensionEncoderTicks = 2234;
    public static final double ticksPerMeter = maxExtensionEncoderTicks / maxExtensionMeters;

    public static final double fullRotationTicks = 1302;
    public static final double ticksPerRadianArm = fullRotationTicks / Math.PI / 2;

    public static final double ticksPerRotation = 42; //real number
    public static final double ticksPerRadianMotor = ticksPerRotation / Math.PI / 2;


}
