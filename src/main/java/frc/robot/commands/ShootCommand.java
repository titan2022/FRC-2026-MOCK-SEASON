package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.base.VoltageControlledBase;

/** One shot using elapsed time, with power latched at the start of the shot. */
public class ShootCommand extends SequentialCommandGroup {
  private double power;
  private String phase = "Idle";

  public ShootCommand(VoltageControlledBase intake, VoltageControlledBase shooter,
      DoubleSupplier powerSupplier) {
    addCommands(Commands.sequence(
        Commands.runOnce(() -> {
          phase = "Starting";
          intake.stop();
          shooter.stop();
          double requested = powerSupplier.getAsDouble();
          power = Double.isFinite(requested) ? MathUtil.clamp(requested, 0.0, 1.0) : 0.0;
        }),
        Commands.runOnce(() -> phase = "Backing ball"),
        intake.reverseCommand().withTimeout(Constants.SHOT_REVERSE_SECONDS)
            .deadlineFor(shooter.reverseCommand()),
        Commands.runOnce(() -> phase = "Spin-up timer"),
        Commands.waitSeconds(Constants.SHOOTER_SPINUP_SECONDS)
            .deadlineFor(intake.setVoltageCommand(0),
                shooter.runEnd(() -> shooter.setVoltage(Constants.MAX_FIREPOWER * power), shooter::stop)),
        Commands.runOnce(() -> phase = "Feeding"),
        intake.setVoltageCommand(Constants.FEED_VOLTAGE).alongWith(
            shooter.runEnd(() -> shooter.setVoltage(Constants.MAX_FIREPOWER * power), shooter::stop))
            .withTimeout(Constants.SHOT_FEED_SECONDS))
        .finallyDo(interrupted -> {
          intake.stop();
          shooter.stop();
          phase = interrupted ? "Interrupted" : "Complete";
        }));
  }

  public double getPower() {
    return power;
  }

  public String getPhase() {
    return phase;
  }
}
