package frc.robot.subsystems.elevator;

public interface ElevatorIO {
  public static class ElevatorIOInputs {
    public boolean frontMotorConnected = false;
    public boolean backMotorConnected = false;
    
    public double frontMotorPositionRad = 0.0;
    public double frontMotorVelocityRadPerSec = 0.0;
    public double frontMotorAppliedVolts = 0.0;
    public double frontMotorCurrentAmps = 0.0;

    public double backMotorPositionRad = 0.0;
    public double backMotorVelocityRadPerSec = 0.0;
    public double backMotorAppliedVolts = 0.0;
    public double backMotorCurrentAmps = 0.0;

    public double targetHeightTicks = 0.0;
    public double targetHeightMeters = 0.0;
    public double currentHeightMeters = 0.0;
    public double currentHeightTicks = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double elevatorFFvolts = 0.0;
    public Object targetArmAngleRadians;
    public Object targetArmAngleTicks;
    public Object currentArmAngleRadians;
    public Object currentArmAngleTicks;
  }

  // Update inputs (called every loop)
  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setElevatorHeightMeters(double height){}

  public default void setElevatorHeightEncoderTicks(double ticks){}

  public default void setArmAngleRad(double radians){}

  public default void setArmAngleEncoderTicks(double ticks){}
}

