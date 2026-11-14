package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.drive.KitbotTankDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ScoreAutoCommand extends SequentialCommandGroup {
  public ScoreAutoCommand(KitbotTankDrivetrain drivetrain, IntakeSubsystem intake, ShooterSubsystem shooter) {
    addCommands(
        drivetrain.driveForward(Constants.AUTO_DRIVE_POWER).withTimeout(Constants.AUTO_DRIVE_SECONDS),
        Commands.waitSeconds(Constants.AUTO_SETTLE_SECONDS),
        new ShootCommand(intake, shooter, () -> Constants.AUTO_SHOOTER_POWER));
  }
}
