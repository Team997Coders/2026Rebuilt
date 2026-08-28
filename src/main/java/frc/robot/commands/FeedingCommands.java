package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.SpIndexer.SpIndexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.AllianceFlipUtil;

public class FeedingCommands {
    public static boolean shouldFeed(Turret turret, Hood hood, BooleanSupplier override) {
        return (!FieldConstants.Regions.behindOpposingHub.contains(AllianceFlipUtil.apply(
                                RobotState.getInstance().getTurretPose().getTranslation()))
                        && !FieldConstants.Regions.underAllianceTower.contains(AllianceFlipUtil.apply(
                                RobotState.getInstance().getTurretPose().getTranslation()))
                        && !RobotState.getInstance().shouldLowerHood()
                        && turret.atSetpoint(Rotation2d.fromDegrees(30.0))
                        && hood.atSetpoint())
                || override.getAsBoolean();
    }

    public static Command feedCommand(Turret turret, Hood hood, SpIndexer spindexer) {
        return feedCommand(turret, hood, spindexer, () -> false);
    }

    public static Command feedCommand(Turret turret, Hood hood, SpIndexer spindexer, BooleanSupplier override) {
        return spindexer
                .runCommand(() -> shouldFeed(turret, hood, override) ? 0.50767676767 : 0.0)
                .alongWith(Commands.run(() -> {
                    boolean shouldFeed = shouldFeed(turret, hood, override);
                    Logger.recordOutput("Spindexer/IsFeeding", shouldFeed);
                    Logger.recordOutput("Spindexer/FeedOverride", override.getAsBoolean() && !shouldFeed);
                }))
                .withName("FeedShooterCommand");
    }
}
