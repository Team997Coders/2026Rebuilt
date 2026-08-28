package frc.robot.subsystems.SpIndexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpIndexerIO {
  @AutoLog
  class SpIndexerIOInputs {
    public boolean spinMotorConnected = false;
    public double spinVelocityRPS = 0.0;
    public double spinAppliedVolts = 0.0;
    public double spinSupplyCurrentAmps = 0.0;
    public double spinStatorCurrentAmps = 0.0;
    public double spinTempCelsius = 0.0;
  }

  default void updateInputs(SpIndexerIOInputs inputs) {}

  default void setMotorVoltages(double spinVolts) {}
}
