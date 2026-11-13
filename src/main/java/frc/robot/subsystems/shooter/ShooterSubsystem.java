package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
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
  public Command fireCommand(DoubleSupplier power) {
    return runEnd(
        // be careful with your lambda!
        () -> setVoltage(Constants.MAX_FIREPOWER * power.getAsDouble()),
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
