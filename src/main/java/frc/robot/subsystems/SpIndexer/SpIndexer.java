package frc.robot.subsystems.SpIndexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SimState;
import frc.robot.util.LoggedTracer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class SpIndexer extends SubsystemBase {
  private static boolean hasInstance = false;

  private final SpIndexerIO io;
  private final SpIndexerIOInputsAutoLogged inputs = new SpIndexerIOInputsAutoLogged();

  private final Alert spinMotorDisconnectedAlert =
      new Alert("Spindexer motor disconnected", Alert.AlertType.kError);

  private SpIndexer(SpIndexerIO io) {
    if (hasInstance) throw new IllegalStateException("Instance of spindexer already exists");
    hasInstance = true;
    this.io = io;
  }

  @Override
  public void periodic() {
    LoggedTracer.reset();

    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);

    if (DriverStation.isDisabled()) stop();

    if (Constants.currentMode == Constants.Mode.SIM) {
      SimState.getInstance().addSpindexerData(inputs.spinAppliedVolts);
    }

    spinMotorDisconnectedAlert.set(!inputs.spinMotorConnected);

    Command activeCmd = CommandScheduler.getInstance().requiring(this);
    Logger.recordOutput(
        "Spindexer/ActiveCommand",
        activeCmd != null
            ? activeCmd.getName() + "_" + Integer.toHexString(activeCmd.hashCode())
            : "None");

    LoggedTracer.record("Spindexer");
  }

  public void setMotorSpeeds(double spinPercent) {
    io.setMotorVoltages(spinPercent * 12.0);
  }

  public void stop() {
    setMotorSpeeds(0.0);
  }

  public Command runCommand(
      DoubleSupplier spinPercent, DoubleSupplier feederPercent, DoubleSupplier feeder2Percent) {
    return runEnd(() -> setMotorSpeeds(spinPercent.getAsDouble()), this::stop)
        .withName("SpindexerRunCommand");
  }

  public static SpIndexer createReal() {
    return new SpIndexer(new SpIndexerIOTalonFX());
  }

  public static SpIndexer createDummy() {
    return new SpIndexer(new SpIndexerIO() {});
  }

  public static SpIndexer createSim() {
    if (Constants.currentMode == Constants.Mode.REAL) {
      DriverStation.reportWarning("Using simulated spindexer on real robot", false);
    }
    return new SpIndexer(new SpIndexerIOSim());
  }
}
