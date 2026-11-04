package frc.robot.subsystems.ball;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.base.VoltageControlledBase;

/** Subsystem Shell
 */
public class BallMechanism extends VoltageControlledBase {
  {
    SUBSYSTEM_NAME = "Ball";
    // Need to assign motor, motorConfig and DEFAULT_VOLTAGE after the hardware decision.
  }

  public BallMechanism() {
    initialize();
    setDefaultCommand(stopCommand());
  }

  @Override
  protected void initialize() {
    if (motor == null || motorConfig == null) {
      throw new IllegalStateException("Ball hardware is unconfigured");
    }
    super.initialize();
  }

  @Override
  public Command setVoltageCommand(double voltage) {
    return run(this::stop);
  }

  public Command captureCommand() { return stopCommand(); }
  public Command releaseCommand() { return stopCommand(); }

  // Persistent idle
  @Override
  public Command stopCommand() { return run(this::stop); }
}
