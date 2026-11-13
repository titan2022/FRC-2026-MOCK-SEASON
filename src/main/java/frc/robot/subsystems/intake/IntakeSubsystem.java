
package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.base.VoltageControlledBase;

public class IntakeSubsystem extends VoltageControlledBase {

  {
    SUBSYSTEM_NAME = "Intake";
    // the front motor drives both intake rollers
    // linked by belt
    // direction may need to be checked on the bot

    motor = new WPI_TalonSRX(3);
    motorConfig = new TalonSRXConfiguration();

    // may need to change
    DEFAULT_VOLTAGE = 12.0;
    // direction
    motor.setInverted(false);

    motor.setNeutralMode(NeutralMode.Brake);
  }
  WPI_TalonSRX shooterMot = new WPI_TalonSRX(2);

  public IntakeSubsystem() {
    initialize();
  }



  // Ani: Implement intake behavior.
public Command intakeCommand() {
    return runEnd(
        () -> {
            shooterMot.setVoltage(-5);
            forward();
        },
        this::stop
    );
}
}
