package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// i forgot to discuss it previously
// but we also have a commands/ dir that we use
// for more complex command chains
// so that they dont gunk up robot.java



public class ShootCommand extends SequentialCommandGroup {
    // power of shot
  private double power;

  public ShootCommand(IntakeSubsystem intake, ShooterSubsystem shooter, DoubleSupplier powerSupplier) {
    addCommands(
        Commands.runOnce(() -> power = powerSupplier.getAsDouble()),
        Commands.sequence(
            // reverse the intake
            intake.reverseCommand().withTimeout(Constants.SHOT_REVERSE_SECONDS)
                .deadlineFor(shooter.reverseCommand()),
            // spin up the primary shooter
            Commands.waitSeconds(Constants.SHOOTER_SPINUP_SECONDS)
                .deadlineFor(intake.setVoltageCommand(0), shooter.fireCommand(() -> power)),
            // run the intake, launching ball into spun-up primary shooter
            intake.feedCommand().alongWith(shooter.fireCommand(() -> power))
                .withTimeout(Constants.SHOT_FEED_SECONDS)));
  }
}
