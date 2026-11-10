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
import frc.robot.subsystems.shooter.ShooterSubsystem;

/** Reduced version of 2026 Robot.java. */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public final CommandXboxController driveController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  // TODO: configure hardware and the controller-specific bases first.
  // public final KitbotTankDrivetrain drivetrain = new KitbotTankDrivetrain();
  // public final BallMechanism ball = new BallMechanism();
  public final ShooterSubsystem shooter = new ShooterSubsystem(null); // Get a motor later...

  public SendableChooser<Command> autoChooser;

  public Robot() {
    DogLog.setEnabled(false);

    configureBindings();
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Do nothing", Commands.none());
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putString("Hardware/Status", "UNCONFIGURED");
  }

  public void configureBindings() {
    // TODO: after configuration, use the same binding pattern as the team Robot.
    // ball.setDefaultCommand(ball.stopCommand());
    // operatorController.rightBumper().whileTrue(ball.captureCommand());
    // operatorController.rightTrigger().whileTrue(ball.releaseCommand());

    operatorController.rightTrigger(0.01).whileTrue(shooter.fireCommand(operatorController));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    // TODO: add stop/brake calls for each subsystem.
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
