package frc.robot;

/** Robot controls. still need to do max fpower */
public final class Constants {
  private Constants() {
  }

  public static final double AUTO_SECONDS = 15.0;
  public static final double TRANSITION_SECONDS = 3.0;
  public static final double TELEOP_SECONDS = 135.0;

  public static final double MAX_FIREPOWER = 12.0; // volts need to tune on the robot
  public static final double FEED_VOLTAGE = 6.0;
  public static final double SHOT_REVERSE_SECONDS = 0.2;
  public static final double SHOOTER_SPINUP_SECONDS = 0.6;
  public static final double SHOT_FEED_SECONDS = 1.0;

  public static final double DRIVE_DEADBAND = 0.08;
  public static final double AIM_DEADBAND = 0.15;
  public static final double AIM_MAX_OUTPUT = 0.40;
}
