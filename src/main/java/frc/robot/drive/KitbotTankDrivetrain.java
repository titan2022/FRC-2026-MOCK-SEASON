// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KitbotTankDrivetrain extends SubsystemBase implements AutoCloseable {
  private final MotorController leftDrive;
  private final MotorController rightDrive;
  private final DifferentialDrive drive;
  private double leftRequestedOutput;
  private double rightRequestedOutput;

  public KitbotTankDrivetrain() {
    this(configureMotor(13, false), configureMotor(15, true));
  }

  /** Package-private injection point for motor-free drivetrain tests. */
  KitbotTankDrivetrain(MotorController leftDrive, MotorController rightDrive) {
    this.leftDrive = leftDrive;
    this.rightDrive = rightDrive;
    // Logical duty cycles before controller inversion, not measured speed.
    drive = new DifferentialDrive(
        output -> {
          leftRequestedOutput = output;
          this.leftDrive.set(output);
        },
        output -> {
          rightRequestedOutput = output;
          this.rightDrive.set(output);
        });
    drive.setDeadband(0.0);
    drive.setMaxOutput(Constants.DRIVE_MAX_OUTPUT);
  }

  private static WPI_TalonSRX configureMotor(int deviceId, boolean inverted) {
    WPI_TalonSRX motor = new WPI_TalonSRX(deviceId);
    motor.configFactoryDefault();
    motor.setInverted(inverted);
    motor.setNeutralMode(NeutralMode.Brake);
    return motor;
  }

  public Command driveArcade(DoubleSupplier speed, DoubleSupplier rotation) {
    return runEnd(() -> arcadeDrive(speed.getAsDouble(), rotation.getAsDouble()), this::brake);
  }

  /** Preserve arcade controls, with bounded inputs before deadband calculation. */
  public void arcadeDrive(double speed, double rotation) {
    if (!Double.isFinite(speed) || !Double.isFinite(rotation)) {
      brake();
      return;
    }
    double forward = MathUtil.applyDeadband(
        MathUtil.clamp(speed, -1.0, 1.0), Constants.DRIVE_DEADBAND);
    double turn = MathUtil.applyDeadband(
        MathUtil.clamp(rotation, -1.0, 1.0), Constants.AIM_DEADBAND);
    drive.setMaxOutput(Constants.DRIVE_MAX_OUTPUT);
    drive.arcadeDrive(forward, turn, false);
  }

  /**
   * Actual logical motor duty cycles, with no joystick deadband, squaring,
   * or teleop multiplier. Positive values request forward on both sides.
   * Call every scheduler cycle so DifferentialDrive MotorSafety remains active.
   */
  public void driveAutonomous(double left, double right) {
    if (!Double.isFinite(left) || !Double.isFinite(right)) {
      brake();
      return;
    }
    drive.setMaxOutput(1.0);
    drive.tankDrive(MathUtil.clamp(left, -1.0, 1.0), MathUtil.clamp(right, -1.0, 1.0), false);
  }

  /** Retain the baseline compatibility alias; its second supplier is unused. */
  public Command driveTank(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed, DoubleSupplier aim) {
    return driveArcade(leftSpeed, aim);
  }

  public double getLeftRequestedOutput() {
    return leftRequestedOutput;
  }

  public double getRightRequestedOutput() {
    return rightRequestedOutput;
  }

  public void brake() {
    drive.stopMotor();
  }

  /** Release simulation/test resources after the drivetrain is no longer in use. */
  @Override
  public void close() {
    brake();
    drive.setSafetyEnabled(false);
    drive.close();
    if (leftDrive instanceof WPI_TalonSRX talon) {
      talon.close();
    }
    if (rightDrive instanceof WPI_TalonSRX talon) {
      talon.close();
    }
  }
}
