// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ScoreAutoCommand;
import frc.robot.drive.KitbotTankDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/** Reduced version of 2026 Robot.java. */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private double shooterPower = 1.0;

  public final CommandXboxController driveController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  // TODO pls double check these
  public final KitbotTankDrivetrain drivetrain = new KitbotTankDrivetrain();
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final ShooterSubsystem shooter = new ShooterSubsystem(); // no need to pass through

  private final ShootCommand shootCommand = new ShootCommand(intake, shooter, () -> shooterPower);

  public SendableChooser<Command> autoChooser;

  public Robot() {
    DogLog.setEnabled(false);
    // fix tunable override
    DogLog.setOptions(DogLog.getOptions().withNtTunables(false));

    configureBindings();
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Do nothing", Commands.none());
    autoChooser.addOption("Drive forward", drivetrain.driveForward(Constants.AUTO_DRIVE_POWER)
        .withTimeout(Constants.AUTO_DRIVE_SECONDS).withTimeout(Constants.AUTO_SECONDS));
    autoChooser.addOption("Drive and shoot", new ScoreAutoCommand(drivetrain, intake, shooter)
        .withTimeout(Constants.AUTO_SECONDS));
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putString("Hardware/Status", "ok u should prob double check");
  }

  // At the moment these are the controlls:
  // If you are to update the behaviors please edit this msg
  // - cinna

  /*
   * Driver left stick Y
   * Arcade drive, scaled to 60% output
   *
   * Driver left/right stick X and operator left stick X
   * Combined steering past a 13% deadband,
   * Scaled to 60% output
   * driver remains active
   * Operator left shoulder
   * Runs the intake/feed rollers
   * Operator right shoulder
   * Reverses those rollers
   * Operator X/Y/A/B
   * Selects 80%/90%/100%/60% power
   * Operator right trigger
   * Fires one timed shot at the selected power
   */

  public void configureBindings() {
    // TODO: after configuration, use the same binding pattern as the team Robot.
    drivetrain.setDefaultCommand(drivetrain.driveArcade(
        // Driver controls forward/backward
        () -> isTeleopEnabled() ? -driveController.getLeftY() : 0,
        // Combine driver left stick, driver right stick, and operator steering
        () -> isTeleopEnabled()
            ? -(driveController.getLeftX() + driveController.getRightX() + operatorController.getLeftX())
            : 0));

    // maybe we make intake spin up by default ? kinda
    // would conflict with the intake mech though
    intake.setDefaultCommand(intake.stopCommand());
    shooter.setDefaultCommand(shooter.stopCommand());

    // chaining triggers
    Trigger teleop = new Trigger(this::isTeleopEnabled);
    Trigger outtake = teleop.and(operatorController.rightBumper());
    // when left bumper AND not outaking run intake
    Trigger capture = teleop.and(operatorController.leftBumper()).and(outtake.negate())
        .and(() -> !shootCommand.isScheduled());
    capture.whileTrue(intake.intakeCommand());
    capture.whileTrue(shooter.reverseCommand());

    // MECHANISM: OUTTAKING
    outtake.whileTrue(intake.reverseCommand());

    teleop.and(operatorController.x()).onTrue(Commands.runOnce(() -> shooterPower = 0.8));
    teleop.and(operatorController.y()).onTrue(Commands.runOnce(() -> shooterPower = 0.9));
    teleop.and(operatorController.a()).onTrue(Commands.runOnce(() -> shooterPower = 1.0));
    teleop.and(operatorController.b()).onTrue(Commands.runOnce(() -> shooterPower = 0.6));
    operatorController.rightTrigger().onTrue(Commands.runOnce(() -> {
      if (isTeleopEnabled() && !outtake.getAsBoolean() && !shootCommand.isScheduled()) {
        shootCommand.schedule();
      }
    }));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Battery voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Shooter/Selected power percent", shooterPower * 100);
    SmartDashboard.putBoolean("Auto/Running",
        m_autonomousCommand != null && m_autonomousCommand.isScheduled());
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    drivetrain.brake();
    intake.stop();
    shooter.stop();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = autoChooser.getSelected();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
