package frc.robot.dashboard;

import static org.junit.jupiter.api.Assertions.*;

import java.util.HashMap;
import java.util.Map;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriverDashboardTest {
  private static final class Capture implements DriverDashboard.Telemetry {
    final Map<String, Object> values = new HashMap<>();

    @Override
    public void text(String key, String value) {
      values.put(key, value);
    }

    @Override
    public void number(String key, double value) {
      values.put(key, value);
    }

    @Override
    public void flag(String key, boolean value) {
      values.put(key, value);
    }

    Object get(String key) {
      return values.get("Driver/" + key);
    }
  }

  private Capture telemetry;
  private DriverDashboard dashboard;
  private DriverDashboard.RobotStatus status;
  private double time;

  @BeforeEach
  void before() {
    telemetry = new Capture();
    status = new DriverDashboard.RobotStatus("TELEOP", true, true, true, true,
        false, 12.3, false, 125.0);
    time = 0.0;
    dashboard = new DriverDashboard(telemetry, () -> status, () -> time);
  }

  private static DriverDashboard.Snapshot snapshot(ShooterPreset preset,
      boolean active, double power, String phase) {
    return new DriverDashboard.Snapshot(preset, active, power, phase,
        "Cross and score", "Cross and score", "Feeding shot", "Running Cross and score",
        3.2, 4.02, true, 0.0, 0.0, 6.0, 9.6);
  }

  @Test
  void preservesAllExistingPresetMappings() {
    assertEquals(0.8, ShooterPreset.X.power());
    assertEquals(0.9, ShooterPreset.Y.power());
    assertEquals(1.0, ShooterPreset.A.power());
    assertEquals(0.6, ShooterPreset.B.power());
    for (ShooterPreset preset : ShooterPreset.values()) {
      assertEquals(preset.name(), preset.buttonLabel());
      assertEquals((int) Math.round(preset.power() * 100), preset.percentage());
      assertEquals(preset.buttonLabel() + " " + preset.percentage() + "%", preset.label());
    }
  }

  @Test
  void changingNextPresetDoesNotMislabelTheLatchedShot() {
    dashboard.update(snapshot(ShooterPreset.X, true, 0.60, "Feeding"));
    assertEquals("X 80%", telemetry.get("Preset"));
    assertEquals(80.0, telemetry.get("Preset percent"));
    assertEquals("60%", telemetry.get("Active shot power"));
    assertEquals(60.0, telemetry.get("Active shot percent"));
    assertEquals(true, telemetry.get("Preset X"));
    assertEquals(false, telemetry.get("Preset A"));
    assertEquals(true, telemetry.get("Shot active"));
    assertEquals("Feeding", telemetry.get("Shot phase"));

    time = 0.11;
    dashboard.update(snapshot(ShooterPreset.A, true, 0.60, "Feeding"));
    assertEquals("A 100%", telemetry.get("Preset"));
    assertEquals("60%", telemetry.get("Active shot power"));
    assertEquals(false, telemetry.get("Preset X"));
    assertEquals(true, telemetry.get("Preset A"));
  }

  @Test
  void idleClearsActiveShotDisplayAndKeepsSelectedPreset() {
    dashboard.update(snapshot(ShooterPreset.Y, true, 0.80, "Feeding"));
    time = 0.11;
    dashboard.update(snapshot(ShooterPreset.Y, false, 0.80, "Complete"));
    assertEquals("Y 90%", telemetry.get("Preset"));
    assertEquals("Idle", telemetry.get("Active shot power"));
    assertEquals(0.0, telemetry.get("Active shot percent"));
    assertEquals("Idle", telemetry.get("Shot phase"));
    assertEquals(false, telemetry.get("Shot active"));
  }

  @Test
  void keepsRequestedOutputAndTimeBasedAutoStatusExplicit() {
    dashboard.update(new DriverDashboard.Snapshot(ShooterPreset.A, false, 0.0, "Idle",
        "Cross line", "Cross and score", "Shooter spin-up delay", "Running Cross and score",
        2.5, 4.02, true, 0.34, 0.26, -12.0, 0.0));
    assertEquals("Cross line", telemetry.get("Auto selected"));
    assertEquals("Cross and score", telemetry.get("Auto active"));
    assertEquals("Shooter spin-up delay", telemetry.get("Auto phase"));
    assertEquals("2.5 / 4.0 s", telemetry.get("Auto timing"));
    assertEquals(34.0, telemetry.get("Drive left requested percent"));
    assertEquals(26.0, telemetry.get("Drive right requested percent"));
    assertEquals(-12.0, telemetry.get("Intake requested volts"));
    assertEquals("Reverse", telemetry.get("Intake state"));
    assertEquals("Stopped", telemetry.get("Shooter state"));
    assertEquals(true, telemetry.get("Auto tuning valid"));
  }

  @Test
  void doesNotPresentMissingMatchClockAsACompletedMatch() {
    status = new DriverDashboard.RobotStatus("DISABLED", false, true, true, true,
        false, 12.3, false, -1.0);
    dashboard.update(snapshot(ShooterPreset.A, false, 1.0, "Idle"));
    assertEquals("Unavailable", telemetry.get("Match clock"));
    assertEquals(-1.0, telemetry.get("Match time seconds"));
    assertEquals(false, telemetry.get("Match clock available"));
    assertEquals("Unavailable", DriverDashboard.formatMatchTime(Double.NaN));
    assertEquals("Unavailable", DriverDashboard.formatMatchTime(Double.POSITIVE_INFINITY));
    assertEquals("0:00", DriverDashboard.formatMatchTime(0.0));
    assertEquals("1:06", DriverDashboard.formatMatchTime(65.1));
    assertEquals("2:00", DriverDashboard.formatMatchTime(120.0));
  }

  @Test
  void publishesRealConnectionAndBrownoutProblems() {
    status = new DriverDashboard.RobotStatus("E-STOP", false, false, false, false,
        true, 8.0, true, -1.0);
    dashboard.update(new DriverDashboard.Snapshot(ShooterPreset.B, false, 0.0, "Idle",
        "Cross line", "None", "Idle", "Blocked", 0.0, 0.0, false, 0.0, 0.0, 0.0, 0.0));
    String attention = (String) telemetry.get("Attention");
    assertTrue(attention.contains("Emergency stop"));
    assertTrue(attention.contains("Driver Station disconnected"));
    assertTrue(attention.contains("Driver controller missing"));
    assertTrue(attention.contains("Operator controller missing"));
    assertTrue(attention.contains("Brownout"));
    assertTrue(attention.contains("Auto settings invalid"));
    assertEquals(true, telemetry.get("Brownout"));
    assertEquals(true, telemetry.get("Low battery"));
    assertEquals(false, telemetry.get("DS connected"));

    status = new DriverDashboard.RobotStatus("TELEOP", true, true, true, true,
        false, 12.3, false, 125.0);
    time = 0.11;
    dashboard.update(snapshot(ShooterPreset.B, false, 0.0, "Idle"));
    assertEquals("No reported issues", telemetry.get("Attention"));
  }

  @Test
  void publishesAtTenHzAndRecoversIfSimulationClockResets() {
    dashboard.update(snapshot(ShooterPreset.A, false, 0.0, "Idle"));
    assertEquals(1.0, telemetry.get("Update sequence"));
    time = 0.05;
    dashboard.update(snapshot(ShooterPreset.B, false, 0.0, "Idle"));
    assertEquals(1.0, telemetry.get("Update sequence"));
    assertEquals("A 100%", telemetry.get("Preset"));
    time = 0.10;
    dashboard.update(snapshot(ShooterPreset.B, false, 0.0, "Idle"));
    assertEquals(2.0, telemetry.get("Update sequence"));
    assertEquals("B 60%", telemetry.get("Preset"));
    time = -0.5;
    dashboard.update(snapshot(ShooterPreset.X, false, 0.0, "Idle"));
    assertEquals(3.0, telemetry.get("Update sequence"));
    assertEquals("X 80%", telemetry.get("Preset"));
  }
}
