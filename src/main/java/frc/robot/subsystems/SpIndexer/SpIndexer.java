package frc.robot.subsystems.SpIndexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class SpIndexer extends SubsystemBase {
    private static boolean hasInstance = false;

    private final SpIndexerIO io;
    private final SpIndexerIOInputsAutoLogged inputs = new SpIndexerIOInputsAutoLogged();

    private final Alert spinMotorDisconnectedAlert = new Alert("Spindexer motor disconnected", Alert.AlertType.kError);

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

        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            SimState.getInstance()
                    .addSpindexerData(inputs.spinAppliedVolts, inputs.feeder1AppliedVolts, inputs.feeder2AppliedVolts);
        }

        spinMotorDisconnectedAlert.set(!inputs.spinMotorConnected);
        feederMotorDisconnectedAlert.set(!inputs.feederMotorConnected);
        feederMotor2DisconnectedAlert.set(!inputs.feeder2MotorConnected);

        Command activeCmd = CommandScheduler.getInstance().requiring(this);
        Logger.recordOutput(
                "Spindexer/ActiveCommand",
                activeCmd != null ? activeCmd.getName() + "_" + Integer.toHexString(activeCmd.hashCode()) : "None");

        LoggedTracer.record("Spindexer");
    }

    public void setMotorSpeeds(double spinPercent, double feederPercent, double feeder2Percent) {
        io.setMotorVoltages(spinPercent * 12.0, feederPercent * 12.0, feeder2Percent * 12.0);
    }

    public void stop() {
        setMotorSpeeds(0.0, 0.0, 0.0);
    }

    public Command runCommand(DoubleSupplier spinPercent, DoubleSupplier feederPercent, DoubleSupplier feeder2Percent) {
        return runEnd(
                        () -> setMotorSpeeds(
                                spinPercent.getAsDouble(), feederPercent.getAsDouble(), feeder2Percent.getAsDouble()),
                        this::stop)
                .withName("SpindexerRunCommand");
    }

    public static Spindexer createReal() {
        return new Spindexer(new SpindexerIOTalonFX());
    }

    public static Spindexer createDummy() {
        return new Spindexer(new SpindexerIO() {});
    }

    public static Spindexer createSim() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated spindexer on real robot", false);
        }
        return new Spindexer(new SpindexerIOSim());
    }
}