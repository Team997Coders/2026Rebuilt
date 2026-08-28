package frc.robot.subsystems.SpIndexer;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.SpIndexer.SpIndexerConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.SimState;
import org.ironmaple.simulation.motorsims.SimulatedBattery;

public class SpIndexerIOSim implements SpIndexerIO {
  private final DCMotorSim spinSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, SPIN_GEAR_RATIO),
          DCMotor.getKrakenX60Foc(1));

  private double spinRequestedVolts = 0.0;
  private double spinAppliedVolts = 0.0;

  public SpIndexerIOSim() {
    SimState.getInstance().addCurrentDraw(spinSim::getCurrentDrawAmps, () -> spinAppliedVolts);
  }

  @Override
  public void updateInputs(SpIndexerIOInputs inputs) {
    spinAppliedVolts = SimulatedBattery.clamp(Volts.of(spinRequestedVolts)).in(Volts);

    spinSim.setInputVoltage(spinAppliedVolts);
    spinSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.spinMotorConnected = true;
    inputs.spinVelocityRPS = spinSim.getAngularVelocityRPM() / 60.0;
    inputs.spinAppliedVolts = spinAppliedVolts;
    inputs.spinStatorCurrentAmps = spinSim.getCurrentDrawAmps();
    inputs.spinSupplyCurrentAmps =
        spinSim.getCurrentDrawAmps()
            * spinAppliedVolts
            / SimulatedBattery.getBatteryVoltage().in(Volts);
  }

  @Override
  public void setMotorVoltages(double spinVolts) {
    spinRequestedVolts = spinVolts;
  }
}
