package frc.robot.subsystems.SpIndexer;

import static frc.robot.subsystems.SpIndexer.SpIndexerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class SpIndexerIOTalonFX implements SpIndexerIO {
  private final TalonFX spinMotor1 = new TalonFX(SPIN_MOTOR_ID);

  private final StatusSignal<AngularVelocity> spinVelocity = spinMotor1.getVelocity();
  private final StatusSignal<Voltage> spinVoltage = spinMotor1.getMotorVoltage();
  private final StatusSignal<Current> spinSupplyCurrent = spinMotor1.getSupplyCurrent();
  private final StatusSignal<Current> spinStatorCurrent = spinMotor1.getStatorCurrent();
  private final StatusSignal<Temperature> spinTemp = spinMotor1.getDeviceTemp();

  private final VoltageOut spinVoltageCtrlReq = new VoltageOut(0).withEnableFOC(true);

  public SpIndexerIOTalonFX() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.SensorToMechanismRatio = SPIN_GEAR_RATIO;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 60.0;
    config.CurrentLimits.SupplyCurrentLowerLimit = 30.0;
    config.CurrentLimits.SupplyCurrentLowerTime = 0.1;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60.0;

    spinMotor1.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, spinVelocity, spinVoltage, spinSupplyCurrent, spinStatorCurrent, spinTemp);

    spinMotor1.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(SpIndexerIOInputs inputs) {
    inputs.spinMotorConnected =
        BaseStatusSignal.refreshAll(
                spinVelocity, spinVoltage, spinSupplyCurrent, spinStatorCurrent, spinTemp)
            .isOK();
    inputs.spinVelocityRPS = spinVelocity.getValueAsDouble();
    inputs.spinAppliedVolts = spinVoltage.getValueAsDouble();
    inputs.spinSupplyCurrentAmps = spinSupplyCurrent.getValueAsDouble();
    inputs.spinStatorCurrentAmps = spinStatorCurrent.getValueAsDouble();
    inputs.spinTempCelsius = spinTemp.getValueAsDouble();
  }

  @Override
  public void setMotorVoltages(double spinVolts) {
    spinMotor1.setControl(spinVoltageCtrlReq.withOutput(spinVolts));
  }
}
