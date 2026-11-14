package frc.robot.auto;

/** Actuator requests, separated from CAN hardware for scheduler tests. */
public interface AutoIO {
  void drive(double leftDuty, double rightDuty);

  void intake(double volts);

  void shooter(double volts);

  void stop();
}
