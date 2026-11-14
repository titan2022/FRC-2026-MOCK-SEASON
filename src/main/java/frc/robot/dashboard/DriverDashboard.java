package frc.robot.dashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Read-only robot telemetry for the checked-in Elastic driver layout.
 *
 * <p>Requested output is not a measured speed, and timed spinup is not an RPM-ready
 * signal. This class never writes to a motor or schedules a command.
 */
public final class DriverDashboard {
  public static final double UPDATE_PERIOD_SECONDS = 0.10;
  public static final double LOW_BATTERY_VOLTS = 10.0;

  /** Values supplied after the scheduler has run. Auto shot power is configured separately. */
  public record Snapshot(
      ShooterPreset selectedPreset,
      boolean shotActive,
      double activeShotPower,
      String shotPhase,
      String selectedAuto,
      String activeAuto,
      String autoPhase,
      String autoStatus,
      double autoElapsedSeconds,
      double autoPlannedSeconds,
      boolean autoTuningValid,
      double leftDriveRequest,
      double rightDriveRequest,
      double intakeVoltageRequest,
      double shooterVoltageRequest) {
    public Snapshot {
      Objects.requireNonNull(selectedPreset, "selectedPreset");
    }
  }

  /** Separated from HAL so dashboard behavior can be tested without robot hardware. */
  record RobotStatus(
      String mode,
      boolean enabled,
      boolean dsConnected,
      boolean driverConnected,
      boolean operatorConnected,
      boolean emergencyStopped,
      double batteryVolts,
      boolean brownedOut,
      double matchTimeSeconds) {
  }

  interface Telemetry {
    void text(String key, String value);

    void number(String key, double value);

    void flag(String key, boolean value);
  }

  private final Telemetry telemetry;
  private final Supplier<RobotStatus> robotStatus;
  private final DoubleSupplier clock;
  private double lastUpdateSeconds = Double.NEGATIVE_INFINITY;
  private int updateSequence;

  public DriverDashboard() {
    this(new Telemetry() {
      @Override
      public void text(String key, String value) {
        SmartDashboard.putString(key, value);
      }

      @Override
      public void number(String key, double value) {
        SmartDashboard.putNumber(key, value);
      }

      @Override
      public void flag(String key, boolean value) {
        SmartDashboard.putBoolean(key, value);
      }
    }, DriverDashboard::readRobotStatus, Timer::getFPGATimestamp);
  }

  DriverDashboard(Telemetry telemetry, Supplier<RobotStatus> robotStatus, DoubleSupplier clock) {
    this.telemetry = Objects.requireNonNull(telemetry);
    this.robotStatus = Objects.requireNonNull(robotStatus);
    this.clock = Objects.requireNonNull(clock);
  }

  /** Call from robotPeriodic after CommandScheduler.run(); publication is limited to 10 Hz. */
  public void update(Snapshot snapshot) {
    Objects.requireNonNull(snapshot, "snapshot");
    double now = clock.getAsDouble();
    if (now >= lastUpdateSeconds && now - lastUpdateSeconds < UPDATE_PERIOD_SECONDS) {
      return;
    }
    lastUpdateSeconds = now;
    RobotStatus robot = robotStatus.get();

    text("Preset", snapshot.selectedPreset().label());
    text("Preset button", snapshot.selectedPreset().buttonLabel());
    number("Preset percent", snapshot.selectedPreset().percentage());
    for (ShooterPreset preset : ShooterPreset.values()) {
      flag("Preset " + preset.buttonLabel(), snapshot.selectedPreset() == preset);
    }
    flag("Shot active", snapshot.shotActive());
    text("Shot phase", snapshot.shotActive() ? known(snapshot.shotPhase()) : "Idle");
    text("Active shot power", snapshot.shotActive()
        ? String.format(Locale.ROOT, "%.0f%%", snapshot.activeShotPower() * 100) : "Idle");
    number("Active shot percent", snapshot.shotActive() ? snapshot.activeShotPower() * 100 : 0);

    text("Auto selected", known(snapshot.selectedAuto()));
    text("Auto active", known(snapshot.activeAuto()));
    text("Auto phase", known(snapshot.autoPhase()));
    text("Auto status", known(snapshot.autoStatus()));
    number("Auto elapsed seconds", snapshot.autoElapsedSeconds());
    number("Auto planned seconds", snapshot.autoPlannedSeconds());
    text("Auto timing", String.format(Locale.ROOT, "%.1f / %.1f s",
        snapshot.autoElapsedSeconds(), snapshot.autoPlannedSeconds()));
    flag("Auto tuning valid", snapshot.autoTuningValid());

    number("Drive left requested percent", snapshot.leftDriveRequest() * 100);
    number("Drive right requested percent", snapshot.rightDriveRequest() * 100);
    number("Intake requested volts", snapshot.intakeVoltageRequest());
    number("Shooter requested volts", snapshot.shooterVoltageRequest());
    text("Intake state", direction(snapshot.intakeVoltageRequest()));
    text("Shooter state", direction(snapshot.shooterVoltageRequest()));

    text("Mode", known(robot.mode()));
    flag("Enabled", robot.enabled());
    flag("DS connected", robot.dsConnected());
    flag("Driver connected", robot.driverConnected());
    flag("Operator connected", robot.operatorConnected());
    flag("Emergency stop", robot.emergencyStopped());
    number("Battery volts", robot.batteryVolts());
    flag("Brownout", robot.brownedOut());
    flag("Low battery", robot.batteryVolts() < LOW_BATTERY_VOLTS);
    boolean clockAvailable = Double.isFinite(robot.matchTimeSeconds()) && robot.matchTimeSeconds() >= 0;
    flag("Match clock available", clockAvailable);
    text("Match clock", formatMatchTime(robot.matchTimeSeconds()));
    number("Match time seconds", clockAvailable ? robot.matchTimeSeconds() : -1);
    text("Attention", attention(robot, snapshot.autoTuningValid()));
    number("Update sequence", ++updateSequence);
  }

  private void text(String key, String value) {
    telemetry.text("Driver/" + key, value);
  }

  private void number(String key, double value) {
    telemetry.number("Driver/" + key, value);
  }

  private void flag(String key, boolean value) {
    telemetry.flag("Driver/" + key, value);
  }

  private static RobotStatus readRobotStatus() {
    String mode = DriverStation.isEStopped() ? "E-STOP"
        : DriverStation.isDisabled() ? "DISABLED"
        : DriverStation.isAutonomous() ? "AUTONOMOUS"
        : DriverStation.isTest() ? "TEST" : "TELEOP";
    return new RobotStatus(mode, DriverStation.isEnabled(), DriverStation.isDSAttached(),
        DriverStation.isJoystickConnected(0), DriverStation.isJoystickConnected(1),
        DriverStation.isEStopped(), RobotController.getBatteryVoltage(), RobotController.isBrownedOut(),
        DriverStation.getMatchTime());
  }

  static String formatMatchTime(double seconds) {
    if (!Double.isFinite(seconds) || seconds < 0) {
      return "Unavailable";
    }
    long remaining = (long) Math.ceil(seconds);
    return String.format(Locale.ROOT, "%d:%02d", remaining / 60, remaining % 60);
  }

  private static String known(String value) {
    return value == null || value.isBlank() ? "None" : value;
  }

  private static String direction(double volts) {
    if (!Double.isFinite(volts)) {
      return "Unknown";
    }
    return volts > 0.05 ? "Forward" : volts < -0.05 ? "Reverse" : "Stopped";
  }

  private static String attention(RobotStatus robot, boolean autoTuningValid) {
    List<String> messages = new ArrayList<>();
    if (robot.emergencyStopped()) {
      messages.add("Emergency stop");
    }
    if (!robot.dsConnected()) {
      messages.add("Driver Station disconnected");
    }
    if (!robot.driverConnected()) {
      messages.add("Driver controller missing");
    }
    if (!robot.operatorConnected()) {
      messages.add("Operator controller missing");
    }
    if (robot.brownedOut()) {
      messages.add("Brownout");
    } else if (robot.batteryVolts() < LOW_BATTERY_VOLTS) {
      messages.add("Battery below 10 V");
    }
    if (!autoTuningValid) {
      messages.add("Auto settings invalid");
    }
    return messages.isEmpty() ? "No reported issues" : String.join(" | ", messages);
  }
}
