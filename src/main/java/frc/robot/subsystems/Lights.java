package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.BooleanSupplier;

public class Lights extends SubsystemBase {
    private final DigitalOutput dio0 = new DigitalOutput(Constants.LightsConstants.dio0Pin);
    private final DigitalOutput dio1 = new DigitalOutput(Constants.LightsConstants.dio1Pin);
    private final DigitalOutput dio2 = new DigitalOutput(Constants.LightsConstants.dio2Pin);
    private final DigitalOutput dio3 = new DigitalOutput(Constants.LightsConstants.dio3Pin);

    private int currentState = 0;

    public void setState(int state) {
        int clamped = Math.max(0, Math.min(15, state));
        currentState = clamped;

        // DIO0 is LSB, DIO3 is MSB.
        dio0.set((clamped & 0x1) != 0);
        dio1.set((clamped & 0x2) != 0);
        dio2.set((clamped & 0x4) != 0);
        dio3.set((clamped & 0x8) != 0);
    }

    public int getState() {
        return currentState;
    }

    public Command holdState(int state) {
        return this.run(() -> setState(state));
    }

    public Command statusIdleRed() {
        return holdState(0);
    }

    public Command statusIdleBlue() {
        return holdState(1);
    }

    public Command statusActiveRed() {
        return holdState(2);
    }

    public Command statusActiveBlue() {
        return holdState(3);
    }

    public Command statusActiveAlliance(BooleanSupplier onBlueAlliance) {
        return this.run(() -> {
            if (onBlueAlliance.getAsBoolean()) {
                setState(3);
            } else {
                setState(2);
            }
        });
    }

    public Command statusByRobotState(BooleanSupplier onBlueAlliance, BooleanSupplier isDisabled) {
        return new Command() {
            {
                addRequirements(Lights.this);
            }

            @Override
            public void execute() {
                if (isDisabled.getAsBoolean()) {
                    setState(0);
                } else {
                    if (onBlueAlliance.getAsBoolean()) {
                        setState(3);
                    } else {
                        setState(2);
                    }
                }
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };
    }

    public Command statusPassing() {
        return holdState(4);
    }

    public Command statusTargetLocked() {
        return holdState(5);
    }

    public Command statusShoot() {
        return holdState(6);
    }

    public Command statusIntaking() {
        return holdState(7);
    }

    public Command statusPurge() {
        return holdState(8);
    }
}
