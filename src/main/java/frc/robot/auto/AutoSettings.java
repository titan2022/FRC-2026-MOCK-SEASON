package frc.robot.auto;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.ToDoubleFunction;

import frc.robot.Constants;

/** Immutable settings captured when autonomous starts. Drive values are actual motor duty. */
public record AutoSettings(
    double startDelaySeconds,
    double crossDriveSeconds,
    double crossDrivePower,
    double driveTrim,
    double scoreApproachSeconds,
    double scoreApproachPower,
    double scoreSettleSeconds,
    double shotReverseSeconds,
    double shotSpinupSeconds,
    double shotFeedSeconds,
    double shotPower,
    double feedVoltage,
    double scoreExitSeconds,
    double scoreExitPower) {

  public static final String PREFIX = "Auto/Tuning/";
  public static final double MAX_RUN_SECONDS = 14.5;
  public static final double MAX_DRIVE_POWER = 0.60;

  /** Starting values only: drive times must be measured on the actual field. */
  public static final AutoSettings DEFAULTS = new AutoSettings(
      0.0, 2.0, 0.30, 0.0,
      2.0, 0.30, 0.25,
      Constants.SHOT_REVERSE_SECONDS, Constants.SHOOTER_SPINUP_SECONDS,
      Constants.SHOT_FEED_SECONDS, 1.0, Constants.FEED_VOLTAGE,
      0.0, 0.30);

  public static Map<String, Double> defaultValues() {
    Map<String, Double> values = new LinkedHashMap<>();
    values.put("Start delay seconds", DEFAULTS.startDelaySeconds());
    values.put("Cross drive seconds", DEFAULTS.crossDriveSeconds());
    values.put("Cross drive power", DEFAULTS.crossDrivePower());
    values.put("Drive trim", DEFAULTS.driveTrim());
    values.put("Score approach seconds", DEFAULTS.scoreApproachSeconds());
    values.put("Score approach power", DEFAULTS.scoreApproachPower());
    values.put("Score settle seconds", DEFAULTS.scoreSettleSeconds());
    values.put("Shot reverse seconds", DEFAULTS.shotReverseSeconds());
    values.put("Shot spinup seconds", DEFAULTS.shotSpinupSeconds());
    values.put("Shot feed seconds", DEFAULTS.shotFeedSeconds());
    values.put("Shot power", DEFAULTS.shotPower());
    values.put("Feed voltage", DEFAULTS.feedVoltage());
    values.put("Score exit seconds", DEFAULTS.scoreExitSeconds());
    values.put("Score exit power", DEFAULTS.scoreExitPower());
    return values;
  }

  public static AutoSettings read(ToDoubleFunction<String> reader) {
    return new AutoSettings(
        reader.applyAsDouble("Start delay seconds"),
        reader.applyAsDouble("Cross drive seconds"),
        reader.applyAsDouble("Cross drive power"),
        reader.applyAsDouble("Drive trim"),
        reader.applyAsDouble("Score approach seconds"),
        reader.applyAsDouble("Score approach power"),
        reader.applyAsDouble("Score settle seconds"),
        reader.applyAsDouble("Shot reverse seconds"),
        reader.applyAsDouble("Shot spinup seconds"),
        reader.applyAsDouble("Shot feed seconds"),
        reader.applyAsDouble("Shot power"),
        reader.applyAsDouble("Feed voltage"),
        reader.applyAsDouble("Score exit seconds"),
        reader.applyAsDouble("Score exit power"));
  }

  public double totalSeconds(AutoMode mode) {
    return switch (mode) {
      case NONE -> 0.0;
      case CROSS -> startDelaySeconds + crossDriveSeconds;
      case SCORE -> startDelaySeconds + scoreApproachSeconds + scoreSettleSeconds
          + shotReverseSeconds + shotSpinupSeconds + shotFeedSeconds + scoreExitSeconds;
    };
  }

  /** Reject invalid values instead of silently clamping a path or substituting a different shot. */
  public List<String> errors(AutoMode mode) {
    List<String> errors = new ArrayList<>();
    if (mode == AutoMode.NONE) {
      return errors;
    }
    range(errors, "Start delay seconds", startDelaySeconds, 0.0, MAX_RUN_SECONDS);
    range(errors, "Drive trim", driveTrim, -0.15, 0.15);
    if (mode == AutoMode.CROSS) {
      duration(errors, "Cross drive seconds", crossDriveSeconds);
      drive(errors, "Cross drive power", crossDrivePower, crossDriveSeconds);
    } else {
      duration(errors, "Score approach seconds", scoreApproachSeconds);
      drive(errors, "Score approach power", scoreApproachPower, scoreApproachSeconds);
      duration(errors, "Score settle seconds", scoreSettleSeconds);
      duration(errors, "Shot reverse seconds", shotReverseSeconds);
      duration(errors, "Shot spinup seconds", shotSpinupSeconds);
      duration(errors, "Shot feed seconds", shotFeedSeconds);
      range(errors, "Shot power", shotPower, 0.0, 1.0);
      range(errors, "Feed voltage", feedVoltage, 0.0, 12.0);
      duration(errors, "Score exit seconds", scoreExitSeconds);
      drive(errors, "Score exit power", scoreExitPower, scoreExitSeconds);
    }
    double total = totalSeconds(mode);
    if (!Double.isFinite(total) || total > MAX_RUN_SECONDS) {
      errors.add("Total auto duration must be at most " + MAX_RUN_SECONDS + " s");
    }
    return List.copyOf(errors);
  }

  private void drive(List<String> errors, String name, double power, double seconds) {
    range(errors, name, power, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
    if (seconds > 0.0 && Double.isFinite(power) && Math.abs(driveTrim) > Math.abs(power)) {
      errors.add(name + " must have magnitude at least as large as trim");
    }
    if (seconds > 0.0 && Double.isFinite(power) && Double.isFinite(driveTrim)
        && (Math.abs(power + driveTrim) > MAX_DRIVE_POWER
            || Math.abs(power - driveTrim) > MAX_DRIVE_POWER)) {
      errors.add(name + " plus trim exceeds " + MAX_DRIVE_POWER + " duty");
    }
  }

  private static void duration(List<String> errors, String name, double seconds) {
    range(errors, name, seconds, 0.0, MAX_RUN_SECONDS);
  }

  private static void range(List<String> errors, String name, double value, double min, double max) {
    if (!Double.isFinite(value) || value < min || value > max) {
      errors.add(name + " must be finite and between " + min + " and " + max);
    }
  }
}
