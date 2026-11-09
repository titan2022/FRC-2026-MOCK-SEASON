
package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final MotorController intakeMotor;
  private final MotorController feederMotor;

  // Cayden, pls config motor controllers and verify which motor
  // drives the shared intake+lower-shooter shaft
  public IntakeSubsystem(MotorController intakeMotor, MotorController feederMotor) {
    this.intakeMotor = intakeMotor;
    this.feederMotor = feederMotor;
  }

  // Ani: Implement intake behavior.
  public Command intakeCommand() {
    return runEnd(() -> {
      // TODO
    }, this::stop);
  }

  // Ani: Implement reversing
  public Command reverseCommand() {
    return runEnd(() -> {
      // TODO
    }, this::stop);
  }

  // Ani/Cayden: Feed a ball into the shooter.
  public Command feedCommand() {
    return runEnd(() -> {
      // TODO: Run the feeder motor.
    }, this::stop);
  }

  public void stop() {
    intakeMotor.stopMotor();
    feederMotor.stopMotor();
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
