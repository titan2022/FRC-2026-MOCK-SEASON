// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.drive.KitbotTankDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/** Reduced version of 2026 Robot.java. */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private boolean shooting;
  private double shooterPower = 1.0;

  public final CommandXboxController driveController = new CommandXboxController(
    0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  // TODO pls double check these
  public final KitbotTankDrivetrain drivetrain = new KitbotTankDrivetrain();
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final ShooterSubsystem shooter = new ShooterSubsystem(); // no need to pass through

  public SendableChooser<Command> autoChooser;

  public Robot() {
    DogLog.setEnabled(false);
    // fix tunable override
    DogLog.setOptions(DogLog.getOptions().withNtTunables(false));

    configureBindings();
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Do nothing", Commands.none());
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putString("Hardware/Status", "ok u should prob double check");
  }

  // At the moment these are the controlls:
  // If you are to update the behaviors please edit this msg
  // - cinna

  /*
   * Driver left/right stick Y
   * Tank drive
   * 
   * Operator left stick X
   * Adds extra turning past a 15% deadband,
   * Capped at 40% output
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
    // ball.setDefaultCommand(ball.stopCommand());
    // operatorController.rightBumper().whileTrue(ball.captureCommand());
    // operatorController.rightTrigger().whileTrue(ball.releaseCommand());

    drivetrain.setDefaultCommand(drivetrain.driveTank(
        // this may be dtm but i think it is "best"
        // after checking for tele, we take into account
        // both the drive and op controller movements
        () -> isTeleopEnabled() ? -driveController.getLeftY() : 0,
        () -> isTeleopEnabled() ? -driveController.getRightY() : 0,
        () -> isTeleopEnabled() ? operatorController.getLeftX() : 0));

    // maybe we make intake spin up by default ? kinda
    // would conflict with the intake mech though
    intake.setDefaultCommand(intake.stopCommand());
    shooter.setDefaultCommand(shooter.stopCommand());

    // chaining triggers
    Trigger teleop = new Trigger(this::isTeleopEnabled);
    Trigger outtake = teleop.and(operatorController.rightBumper());
    // when left bumper AND not outaking run intake
    teleop.and(operatorController.leftBumper()).and(outtake.negate()).and(() -> !shooting)
        .whileTrue(intake.intakeCommand());
    teleop.and(operatorController.leftBumper()).and(outtake.negate()).and(() -> !shooting)
        .whileTrue(shooter.setVoltageCommand(-12));

    // MECHANISM: OUTTAKING
    outtake.whileTrue(intake.reverseCommand());

    teleop.and(operatorController.x()).onTrue(Commands.runOnce(() -> shooterPower = 0.8));
    teleop.and(operatorController.y()).onTrue(Commands.runOnce(() -> shooterPower = 0.9));
    teleop.and(operatorController.a()).onTrue(Commands.runOnce(() -> shooterPower = 1.0));
    teleop.and(operatorController.b()).onTrue(Commands.runOnce(() -> shooterPower = 0.6));
    teleop.and(operatorController.rightTrigger()).and(outtake.negate())
        .onTrue(Commands.defer(() -> shotCommand(shooterPower), java.util.Set.of(intake, shooter)));
  }

  private Command shotCommand(double power) {
    return Commands.sequence(
        intake.reverseCommand().withTimeout(0.2).deadlineFor(shooter.setVoltageCommand(-12)),
        Commands.waitSeconds(0.6).deadlineFor(
            intake.setVoltageCommand(0), shooter.setVoltageCommand(Constants.MAX_FIREPOWER * power)),
        intake.setVoltageCommand(6.0).alongWith(shooter.setVoltageCommand(Constants.MAX_FIREPOWER * power))
            .withTimeout(1.0))
        .beforeStarting(() -> shooting = true)
        .finallyDo(() -> shooting = false);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
