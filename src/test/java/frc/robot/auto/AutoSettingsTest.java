package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.*;

import java.util.LinkedHashMap;
import java.util.Map;
import org.junit.jupiter.api.Test;

class AutoSettingsTest {
  static AutoSettings with(Map<String, Double> changes) {
    Map<String, Double> values = new LinkedHashMap<>(AutoSettings.defaultValues());
    values.putAll(changes);
    return AutoSettings.read(values::get);
  }

  @Test
  void defaultsFitAllModesAndScoringIncludesApproach() {
    for (AutoMode mode : AutoMode.values()) {
      assertTrue(AutoSettings.DEFAULTS.errors(mode).isEmpty());
    }
    assertEquals(0.0, AutoSettings.DEFAULTS.totalSeconds(AutoMode.NONE));
    assertEquals(2.0, AutoSettings.DEFAULTS.totalSeconds(AutoMode.CROSS));
    assertEquals(4.02, AutoSettings.DEFAULTS.totalSeconds(AutoMode.SCORE), 1e-9);
    assertEquals(2.0, AutoSettings.DEFAULTS.scoreApproachSeconds());
    assertEquals(14, AutoSettings.defaultValues().size());
  }

  @Test
  void allUsedNonFiniteSettingsAreRejected() {
    for (String key : AutoSettings.defaultValues().keySet()) {
      AutoMode mode = key.startsWith("Cross") ? AutoMode.CROSS : AutoMode.SCORE;
      for (double invalid : new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
        assertFalse(with(Map.of(key, invalid)).errors(mode).isEmpty(), key);
      }
    }
  }

  @Test
  void negativeDurationsAndOutOfRangePowersAreRejected() {
    for (String key : AutoSettings.defaultValues().keySet()) {
      if (key.endsWith("seconds")) {
        AutoMode mode = key.startsWith("Cross") ? AutoMode.CROSS : AutoMode.SCORE;
        assertFalse(with(Map.of(key, -0.01)).errors(mode).isEmpty(), key);
      }
    }
    assertFalse(with(Map.of("Cross drive power", 0.61)).errors(AutoMode.CROSS).isEmpty());
    assertFalse(with(Map.of("Score approach power", -0.61)).errors(AutoMode.SCORE).isEmpty());
    assertFalse(with(Map.of("Shot power", 1.01)).errors(AutoMode.SCORE).isEmpty());
    assertFalse(with(Map.of("Shot power", -0.01)).errors(AutoMode.SCORE).isEmpty());
    assertFalse(with(Map.of("Feed voltage", 12.01)).errors(AutoMode.SCORE).isEmpty());
    assertFalse(with(Map.of("Feed voltage", -0.01)).errors(AutoMode.SCORE).isEmpty());
  }

  @Test
  void driveTrimCannotExceedLimitOrReverseOneWheel() {
    assertFalse(with(Map.of("Drive trim", 0.16)).errors(AutoMode.CROSS).isEmpty());
    assertFalse(with(Map.of("Cross drive power", 0.60, "Drive trim", 0.01))
        .errors(AutoMode.CROSS).isEmpty());
    assertFalse(with(Map.of("Cross drive power", 0.05, "Drive trim", 0.06))
        .errors(AutoMode.CROSS).isEmpty());
    assertTrue(with(Map.of("Cross drive power", 0.30, "Drive trim", 0.04))
        .errors(AutoMode.CROSS).isEmpty());
    assertTrue(with(Map.of("Cross drive power", -0.30, "Drive trim", 0.04))
        .errors(AutoMode.CROSS).isEmpty());
  }

  @Test
  void rejectsTotalBudgetRatherThanTruncatingScoringSequence() {
    assertFalse(with(Map.of("Start delay seconds", 13.0)).errors(AutoMode.CROSS).isEmpty());
    assertFalse(with(Map.of("Score approach seconds", 12.5)).errors(AutoMode.SCORE).isEmpty());
    assertTrue(with(Map.of("Start delay seconds", 12.5)).errors(AutoMode.CROSS).isEmpty());
  }

  @Test
  void unrelatedShotSettingsDoNotBlockCrossAndNoneAlwaysStops() {
    AutoSettings invalidShot = with(Map.of("Shot power", Double.NaN, "Shot feed seconds", -1.0));
    assertTrue(invalidShot.errors(AutoMode.CROSS).isEmpty());
    assertTrue(invalidShot.errors(AutoMode.NONE).isEmpty());
    assertFalse(invalidShot.errors(AutoMode.SCORE).isEmpty());
    assertTrue(with(Map.of("Start delay seconds", Double.NaN)).errors(AutoMode.NONE).isEmpty());
  }
}
