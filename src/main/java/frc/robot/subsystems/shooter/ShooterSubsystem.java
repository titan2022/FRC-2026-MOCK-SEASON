
package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.base.VoltageControlledBase;

public class ShooterSubsystem extends VoltageControlledBase {
  // init clause
  {
    SUBSYSTEM_NAME = "Shooter";
    // Back motor drives the upper shooter
    // we need to double check this
    // on the robot
    motor = new WPI_TalonSRX(2);
    motorConfig = new TalonSRXConfiguration();
    motorConfig.peakOutputReverse = -1.0;
    DEFAULT_VOLTAGE = Constants.MAX_FIREPOWER;
    motor.setInverted(false);
    // dont want abrupt stop
    motor.setNeutralMode(NeutralMode.Coast);

  }

  public ShooterSubsystem() {
    initialize();
  }

  // yes this is vague sorry
  public Command fireCommand(CommandXboxController operatorController) {
    return runEnd(
        // be careful with your lambda!
        () -> setVoltage(Constants.MAX_FIREPOWER * operatorController.getRightTriggerAxis()),
        this::stop);
  }

  /*
   * remmeber that this is meant to extend
   * VoltageControlledBase ! so we do not need
   * these implemented
   * 
   * public void stop() {
   * shooterMotor.stopMotor();
   * }
   * 
   * public Command stopCommand() {
   * return runOnce(this::stop);
   * }
   */
}
