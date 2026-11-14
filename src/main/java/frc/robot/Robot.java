// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoManager;
import frc.robot.auto.AutoMode;
import frc.robot.commands.ShootCommand;
import frc.robot.dashboard.DriverDashboard;
import frc.robot.dashboard.ShooterPreset;
import frc.robot.drive.KitbotTankDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/** Mock-season robot: arcade teleop, timed autonomous, and driver telemetry. */
public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private ShooterPreset shooterPreset = ShooterPreset.A;

  public final CommandXboxController driveController;
  public final CommandXboxController operatorController;
  public final KitbotTankDrivetrain drivetrain;
  public final IntakeSubsystem intake;
  public final ShooterSubsystem shooter;
  public final SendableChooser<AutoMode> autoChooser;

  private final ShootCommand shootCommand;
  private final AutoManager autos;
  private final DriverDashboard dashboard;

  public Robot() {
    // Configure logging before subsystem construction registers any tunables.
    DogLog.setEnabled(false);
    DogLog.setOptions(DogLog.getOptions().withNtTunables(false));

    driveController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);
    drivetrain = new KitbotTankDrivetrain();
    intake = new IntakeSubsystem();
    shooter = new ShooterSubsystem();
    shootCommand = new ShootCommand(intake, shooter, () -> shooterPreset.power());
    autos = new AutoManager(drivetrain, intake, shooter);
    autoChooser = autos.chooser();
    dashboard = new DriverDashboard();
    configureBindings();
  }

  /*
   * Driver left Y: forward/backward (60% maximum output).
   * Driver left X + right X + operator left X: combined arcade steering.
   * Operator left bumper: intake and reverse shooter for ball capture.
   * Operator right bumper: reverse intake; takes priority over capture/shooting.
   * Operator X/Y/A/B: select 80%/90%/100%/60% for the next teleop shot.
   * Operator right trigger: one timed shot per fresh press, with power latched.
   * Auto power and timings are separate settings under Auto/Tuning.
   */
  public void configureBindings() {
    drivetrain.setDefaultCommand(drivetrain.driveArcade(
        () -> isTeleopEnabled() ? -driveController.getLeftY() : 0.0,
        () -> isTeleopEnabled()
            ? -(driveController.getLeftX() + driveController.getRightX()
                + operatorController.getLeftX())
            : 0.0));
    intake.setDefaultCommand(intake.stopCommand());
    shooter.setDefaultCommand(shooter.stopCommand());

    Trigger teleop = new Trigger(this::isTeleopEnabled);
    Trigger outtake = teleop.and(operatorController.rightBumper());
    Trigger capture = teleop.and(operatorController.leftBumper()).and(outtake.negate())
        .and(() -> !shootCommand.isScheduled());
    capture.whileTrue(intake.intakeCommand());
    capture.whileTrue(shooter.reverseCommand());
    outtake.whileTrue(intake.reverseCommand());

    teleop.and(operatorController.x()).onTrue(Commands.runOnce(() -> shooterPreset = ShooterPreset.X));
    teleop.and(operatorController.y()).onTrue(Commands.runOnce(() -> shooterPreset = ShooterPreset.Y));
    teleop.and(operatorController.a()).onTrue(Commands.runOnce(() -> shooterPreset = ShooterPreset.A));
    teleop.and(operatorController.b()).onTrue(Commands.runOnce(() -> shooterPreset = ShooterPreset.B));

    // Gate inside the rising-edge action: entering teleop with a held trigger
    // must not create a synthetic new press and fire a shot.
    operatorController.rightTrigger().onTrue(Commands.runOnce(() -> {
      if (isTeleopEnabled() && !outtake.getAsBoolean() && !shootCommand.isScheduled()) {
        CommandScheduler.getInstance().schedule(shootCommand);
      }
    }));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    autos.periodic();
    boolean autoShotActive = autos.shotActive();
    dashboard.update(new DriverDashboard.Snapshot(
        shooterPreset, autoShotActive || shootCommand.isScheduled(),
        autoShotActive ? autos.shotPower() : shootCommand.getPower(),
        autoShotActive ? autos.phase() : shootCommand.getPhase(),
        autos.selectedName(), autos.activeName(), autos.phase(), autos.status(),
        autos.elapsedSeconds(), autos.plannedSeconds(), autos.tuningValid(),
        drivetrain.getLeftRequestedOutput(), drivetrain.getRightRequestedOutput(),
        intake.getVoltageSetpoint(), shooter.getVoltageSetpoint()));
  }

  private void stopAllOutputs() {
    drivetrain.brake();
    intake.stop();
    shooter.stop();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    stopAllOutputs();
  }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();
    stopAllOutputs();
    autonomousCommand = autos.createSelectedCommand();
    CommandScheduler.getInstance().schedule(autonomousCommand);
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    autonomousCommand = null;
    stopAllOutputs();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    stopAllOutputs();
  }
}
