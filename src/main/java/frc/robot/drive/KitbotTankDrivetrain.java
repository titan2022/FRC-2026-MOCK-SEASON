// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    
    drive = new DifferentialDrive(leftDrive::set, rightDrive::set);
  }


   // TODO (Lia): set up drive speed limiting and
   // test out the drive behavior s.t. its right
   public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.runEnd(
        () -> drive.arcadeDrive(
            xSpeed.getAsDouble(),
            zRotation.getAsDouble()),
        this::brake);
  }

  public void brake() {
    drive.stopMotor();
  }
}
