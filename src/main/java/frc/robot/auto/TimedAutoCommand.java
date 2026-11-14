package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

/**
 * Sensorless auto. Every phase refreshes every actuator each loop and every exit stops them.
 * Absolute elapsed time prevents accumulated scheduler delays from overrunning the auto budget.
 */
public final class TimedAutoCommand extends Command {
  static final double MAX_LOOP_GAP_SECONDS = 0.10;

  private record Step(String name, double seconds, double left, double right,
      double intakeVolts, double shooterVolts) {}

  private final AutoMode mode;
  private final AutoSettings settings;
  private final AutoIO io;
  private final DoubleSupplier clock;
  private final BooleanSupplier autonomousEnabled;
  private final List<Step> steps;
  private final List<String> errors;
  private double startedAt;
  private double elapsed;
  private boolean finished;
  private String phase = "Not started";
  private String status = "Not started";

  public TimedAutoCommand(AutoMode mode, AutoSettings settings, AutoIO io,
      Subsystem... requirements) {
    this(mode, settings, io, Timer::getFPGATimestamp,
        DriverStation::isAutonomousEnabled, requirements);
  }

  TimedAutoCommand(AutoMode mode, AutoSettings settings, AutoIO io,
      DoubleSupplier clock, BooleanSupplier autonomousEnabled, Subsystem... requirements) {
    this.mode = mode;
    this.settings = settings;
    this.io = io;
    this.clock = clock;
    this.autonomousEnabled = autonomousEnabled;
    this.errors = settings.errors(mode);
    this.steps = buildSteps(mode, settings);
    addRequirements(requirements);
    setName(mode.displayName());
  }

  private static List<Step> buildSteps(AutoMode mode, AutoSettings s) {
    List<Step> result = new ArrayList<>();
    if (mode == AutoMode.NONE || !s.errors(mode).isEmpty()) {
      return result;
    }
    result.add(new Step("Start delay", s.startDelaySeconds(), 0, 0, 0, 0));
    if (mode == AutoMode.CROSS) {
      result.add(driveStep("Crossing", s.crossDriveSeconds(), s.crossDrivePower(), s.driveTrim()));
    } else {
      result.add(driveStep("Approaching scoring position", s.scoreApproachSeconds(),
          s.scoreApproachPower(), s.driveTrim()));
      result.add(new Step("Settling", s.scoreSettleSeconds(), 0, 0, 0, 0));
      // Preserve the existing ShootCommand sequence: reverse BOTH rollers.
      result.add(new Step("Reversing ball", s.shotReverseSeconds(), 0, 0, -12.0, -12.0));
      result.add(new Step("Shooter spin-up delay", s.shotSpinupSeconds(), 0, 0, 0,
          Constants.MAX_FIREPOWER * s.shotPower()));
      result.add(new Step("Feeding shot", s.shotFeedSeconds(), 0, 0, s.feedVoltage(),
          Constants.MAX_FIREPOWER * s.shotPower()));
      result.add(driveStep("Exiting", s.scoreExitSeconds(), s.scoreExitPower(), s.driveTrim()));
    }
    return List.copyOf(result);
  }

  private static Step driveStep(String name, double seconds, double power, double trim) {
    return new Step(name, seconds, power + trim, power - trim, 0, 0);
  }

  @Override
  public void initialize() {
    io.stop();
    startedAt = clock.getAsDouble();
    elapsed = 0.0;
    finished = false;
    phase = "Starting";
    status = "Running " + mode.displayName();
    if (!errors.isEmpty()) {
      stopWith("Invalid settings", "Blocked: " + String.join("; ", errors));
    } else if (!Double.isFinite(startedAt)) {
      stopWith("Clock error", "Stopped: invalid clock");
    } else if (mode == AutoMode.NONE) {
      stopWith("Do nothing", "Do nothing selected");
    } else if (!autonomousEnabled.getAsBoolean()) {
      stopWith("Stopped", "Stopped: autonomous is not enabled");
    }
  }

  @Override
  public void execute() {
    if (finished) {
      return;
    }
    if (!autonomousEnabled.getAsBoolean()) {
      stopWith("Stopped", "Stopped: autonomous is not enabled");
      return;
    }
    double nextElapsed = clock.getAsDouble() - startedAt;
    if (!Double.isFinite(nextElapsed) || nextElapsed < elapsed) {
      stopWith("Clock error", "Stopped: clock moved backwards or became invalid");
      return;
    }
    if (nextElapsed - elapsed > MAX_LOOP_GAP_SECONDS + 1e-9) {
      stopWith("Loop overrun", "Stopped: more than 100 ms between auto updates");
      return;
    }
    elapsed = nextElapsed;
    double end = 0.0;
    for (Step step : steps) {
      end += step.seconds();
      if (elapsed < end && elapsed < AutoSettings.MAX_RUN_SECONDS) {
        phase = step.name();
        io.drive(step.left(), step.right());
        io.intake(step.intakeVolts());
        io.shooter(step.shooterVolts());
        return;
      }
    }
    stopWith("Complete", "Timed routine complete; position and score are not sensor-verified");
  }

  private void stopWith(String phase, String status) {
    this.phase = phase;
    this.status = status;
    finished = true;
    io.stop();
  }

  @Override
  public boolean isFinished() {
    return finished;
  }

  @Override
  public void end(boolean interrupted) {
    io.stop();
    if (interrupted && !finished) {
      phase = "Interrupted";
      status = "Interrupted; all outputs stopped";
    }
    finished = true;
  }

  public String phase() { return phase; }
  public String status() { return status; }
  public double elapsedSeconds() { return elapsed; }
  public double plannedSeconds() { return errors.isEmpty() ? settings.totalSeconds(mode) : 0.0; }
  public AutoMode mode() { return mode; }
  public AutoSettings settings() { return settings; }
}
