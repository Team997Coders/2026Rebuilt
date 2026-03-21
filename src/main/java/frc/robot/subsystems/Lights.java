package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.EnumMap;
import java.util.Set;
import java.util.function.BooleanSupplier;

public class Lights extends SubsystemBase {
    private final DigitalOutput dio0 = new DigitalOutput(Constants.LightsConstants.dio0Pin);
    private final DigitalOutput dio1 = new DigitalOutput(Constants.LightsConstants.dio1Pin);
    private final DigitalOutput dio2 = new DigitalOutput(Constants.LightsConstants.dio2Pin);
    private final DigitalOutput dio3 = new DigitalOutput(Constants.LightsConstants.dio3Pin);

    private int currentState = 0;

    public enum RequestedState {
        PURGE(8, 50),
        SHOOT(6, 40),
        TARGET_LOCKED(5, 30),
        PASSING(4, 20),
        INTAKING(7, 10);

        private final int stateId;
        private final int priority;

        RequestedState(int stateId, int priority) {
            this.stateId = stateId;
            this.priority = priority;
        }
    }

    private final EnumMap<RequestedState, Integer> requestCounts = new EnumMap<>(RequestedState.class);

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

    public void setRequestActive(RequestedState request, boolean active) {
        int count = requestCounts.getOrDefault(request, 0);
        if (active) {
            requestCounts.put(request, count + 1);
        } else if (count > 0) {
            requestCounts.put(request, count - 1);
        }
    }

    private boolean isRequestActive(RequestedState request) {
        return requestCounts.getOrDefault(request, 0) > 0;
    }

    private int resolveState(BooleanSupplier onBlueAlliance, BooleanSupplier isDisabled) {
        if (isDisabled.getAsBoolean()) {
            return 0;
        }

        RequestedState best = null;
        for (RequestedState request : RequestedState.values()) {
            if (!isRequestActive(request)) {
                continue;
            }
            if (best == null || request.priority > best.priority) {
                best = request;
            }
        }

        if (best != null) {
            return best.stateId;
        }

        return onBlueAlliance.getAsBoolean() ? 3 : 2;
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
            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(Lights.this);
            }

            @Override
            public void execute() {
                setState(resolveState(onBlueAlliance, isDisabled));
            }

            @Override
            public boolean isFinished() {
                return false;
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
