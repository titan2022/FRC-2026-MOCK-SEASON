// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Reduced from the existing kitbot branch.
 */
public class KitbotTankDrivetrain extends SubsystemBase {

  public MotorController leftLeader;
  public MotorController rightLeader;
  protected DifferentialDrive drive;

  public KitbotTankDrivetrain() {
    initialize();
  }

  protected void initialize() {
    if (leftLeader == null || rightLeader == null) {
      throw new IllegalStateException("Drive hardware is unconfigured");
    }
    drive = new DifferentialDrive(leftLeader::set, rightLeader::set);
  }

  // Command factory to create command to drive the robot with joystick inputs.
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
  }

  public void brake() {
    drive.stopMotor();
  }
}
