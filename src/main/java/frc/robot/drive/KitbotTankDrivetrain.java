// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Reduced from the existing kitbot branch.
 */
public class KitbotTankDrivetrain extends SubsystemBase {
  // TODO (Aadya): pls check that these CAN IDs and motor directions are right
  public WPI_TalonSRX leftDrive = new WPI_TalonSRX(13);
  public WPI_TalonSRX rightDrive = new WPI_TalonSRX(15);

  protected DifferentialDrive drive;

  public KitbotTankDrivetrain() {
    initialize();
  }

  protected void initialize() {
    leftDrive.configFactoryDefault();
    rightDrive.configFactoryDefault();
    leftDrive.setInverted(false);
    rightDrive.setInverted(true);
    leftDrive.setNeutralMode(NeutralMode.Brake);
    rightDrive.setNeutralMode(NeutralMode.Brake);
    drive = new DifferentialDrive(leftDrive::set, rightDrive::set);
    // this MAY need to be chanegd
    drive.setDeadband(0);
  }

  // TODO (Lia): set up drive speed limiting and
  // test out the drive behavior s.t. its right
  public Command driveTank(DoubleSupplier leftSpeed, DoubleSupplier rightSpeed, DoubleSupplier aim) {
    return this.runEnd(
        () -> {

          // deadbanding
          double left = MathUtil.applyDeadband(leftSpeed.getAsDouble(), Constants.DRIVE_DEADBAND);
          double right = MathUtil.applyDeadband(rightSpeed.getAsDouble(), Constants.DRIVE_DEADBAND);
          // remember, aim is the operator controller's seperate amt
          double turn = MathUtil.applyDeadband(aim.getAsDouble(), Constants.AIM_DEADBAND)
              * Constants.AIM_MAX_OUTPUT;
          drive.tankDrive(left + turn, right - turn, false);
        },
        this::brake);
  }

  public void brake() {
    drive.stopMotor();
  }
}
