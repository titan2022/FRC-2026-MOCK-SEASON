
package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final MotorController shooterMotor;

  // yall will need a map for the firing table but we can do that later

  // Tanush: Configure the independent firing motor
  // and verify its direction and output limits,
  public ShooterSubsystem(MotorController shooterMotor) {
    this.shooterMotor = shooterMotor;

    
  }

  // Bastian: Implement firing

  
  // yes this is vague sorry
  public Command fireCommand(CommandXboxController operatorController) {
    return runEnd(() -> {
      // TODO: Run the shooter motor at the selected output.
      shooterMotor.setVoltage(Constants.MAX_FIREPOWER*operatorController.getRightTriggerAxis());
    }, this::stop);
  }

  public void stop() {
    shooterMotor.stopMotor();
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
