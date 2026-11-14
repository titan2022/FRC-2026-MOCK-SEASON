package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.*;
import java.util.Map;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

class TimedAutoCommandTest {
  private static final class FakeIO implements AutoIO {
    double left, right, intake, shooter;
    int writes;
    public void drive(double leftDuty, double rightDuty) { left = leftDuty; right = rightDuty; writes++; }
    public void intake(double volts) { intake = volts; }
    public void shooter(double volts) { shooter = volts; }
    public void stop() { left = right = intake = shooter = 0.0; }
    void assertStopped() {
      assertEquals(0.0, left);
      assertEquals(0.0, right);
      assertEquals(0.0, intake);
      assertEquals(0.0, shooter);
    }
  }
  private final CommandScheduler scheduler = CommandScheduler.getInstance();
  private FakeIO io;
  private SubsystemBase drive, intake, shooter;
  private double time;

  @BeforeAll
  static void initializeHal() { assertTrue(HAL.initialize(500, 0)); }

  @BeforeEach
  void before() {
    scheduler.cancelAll();
    scheduler.clearComposedCommands();
    scheduler.getActiveButtonLoop().clear();
    scheduler.enable();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    io = new FakeIO();
    drive = new SubsystemBase() {};
    intake = new SubsystemBase() {};
    shooter = new SubsystemBase() {};
    time = 0.0;
  }

  @AfterEach
  void after() {
    scheduler.cancelAll();
    scheduler.unregisterSubsystem(drive, intake, shooter);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
  }

  private TimedAutoCommand auto(AutoMode mode, AutoSettings settings) {
    return new TimedAutoCommand(mode, settings, io, () -> time,
        DriverStation::isAutonomousEnabled, drive, intake, shooter);
  }

  private void at(double seconds) {
    while (time + 0.02 < seconds - 1e-9) {
      time += 0.02;
      scheduler.run();
    }
    time = seconds;
    scheduler.run();
  }

  @Test
  void crossWaitsThenUsesUnsquaredDutyAndTrimThenStops() {
    AutoSettings settings = AutoSettingsTest.with(Map.of(
        "Start delay seconds", 0.50, "Cross drive seconds", 1.0,
        "Cross drive power", 0.30, "Drive trim", 0.04));
    TimedAutoCommand command = auto(AutoMode.CROSS, settings);
    scheduler.schedule(command);
    at(0.49);
    io.assertStopped();
    assertEquals("Start delay", command.phase());
    at(0.50);
    assertEquals(0.34, io.left, 1e-9);
    assertEquals(0.26, io.right, 1e-9);
    assertEquals(0.0, io.intake);
    assertEquals(0.0, io.shooter);
    at(1.49);
    assertTrue(command.isScheduled());
    at(1.50);
    assertFalse(command.isScheduled());
    io.assertStopped();
    assertEquals("Complete", command.phase());
  }

  @Test
  void scoringApproachesSettlesReversesSpinsFeedsThenOptionallyExits() {
    AutoSettings settings = AutoSettingsTest.with(Map.of(
        "Score approach seconds", 1.0, "Score settle seconds", 0.25,
        "Shot reverse seconds", 0.25, "Shot spinup seconds", 0.50,
        "Shot feed seconds", 0.75, "Shot power", 0.8,
        "Score exit seconds", 0.5, "Score exit power", -0.25));
    TimedAutoCommand command = auto(AutoMode.SCORE, settings);
    scheduler.schedule(command);
    at(0.0);
    assertEquals(0.30, io.left);
    assertEquals(0.0, io.shooter);
    at(1.0);
    io.assertStopped();
    assertEquals("Settling", command.phase());
    at(1.25);
    assertEquals(-12.0, io.intake);
    assertEquals(-12.0, io.shooter);
    assertEquals(0.0, io.left);
    at(1.5);
    assertEquals(0.0, io.intake);
    assertEquals(9.6, io.shooter, 1e-9);
    at(2.0);
    assertEquals(6.0, io.intake);
    assertEquals(9.6, io.shooter, 1e-9);
    at(2.75);
    assertEquals(-0.25, io.left);
    assertEquals(0.0, io.intake);
    assertEquals(0.0, io.shooter);
    at(3.25);
    io.assertStopped();
    assertFalse(command.isScheduled());
  }

  @ParameterizedTest
  @ValueSource(doubles = {0.10, 0.75, 1.10, 1.35, 1.75, 2.25, 3.0})
  void interruptionInEveryPhaseStopsAllOutputs(double interruptAt) {
    AutoSettings settings = AutoSettingsTest.with(Map.of(
        "Start delay seconds", 0.5, "Score approach seconds", 0.5,
        "Score settle seconds", 0.25, "Shot reverse seconds", 0.25,
        "Shot spinup seconds", 0.5, "Shot feed seconds", 0.75,
        "Score exit seconds", 0.5));
    TimedAutoCommand command = auto(AutoMode.SCORE, settings);
    scheduler.schedule(command);
    at(interruptAt);
    command.cancel();
    io.assertStopped();
    assertFalse(command.isScheduled());
    assertEquals("Interrupted", command.phase());
  }

  @Test
  void disableCancelsAutoAndStopsAllOutputs() {
    TimedAutoCommand command = auto(AutoMode.SCORE, AutoSettings.DEFAULTS);
    scheduler.schedule(command);
    at(3.10);
    assertTrue(io.intake > 0.0);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    at(3.12);
    io.assertStopped();
    assertFalse(command.isScheduled());
  }

  @Test
  void modeChangeStopsEvenBeforeRobotTeleopInitCancelsCommand() {
    TimedAutoCommand command = auto(AutoMode.CROSS, AutoSettings.DEFAULTS);
    scheduler.schedule(command);
    at(0.25);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    at(0.27);
    io.assertStopped();
    assertFalse(command.isScheduled());
  }

  @Test
  void noneAndInvalidSettingsNeverProduceMotorOutputs() {
    for (AutoMode mode : new AutoMode[] {AutoMode.NONE, AutoMode.CROSS, AutoMode.SCORE}) {
      TimedAutoCommand command = auto(mode,
          AutoSettingsTest.with(Map.of("Start delay seconds", Double.NaN)));
      scheduler.schedule(command);
      at(0.0);
      io.assertStopped();
      assertEquals(0, io.writes);
      assertFalse(command.isScheduled());
    }
  }

  @Test
  void zeroDurationPhasesAreSkippedWithoutMotorPulse() {
    AutoSettings settings = AutoSettingsTest.with(Map.of(
        "Score approach seconds", 0.0, "Score settle seconds", 0.0,
        "Shot reverse seconds", 0.0, "Shot spinup seconds", 0.0,
        "Shot feed seconds", 0.0, "Score exit seconds", 0.0));
    TimedAutoCommand command = auto(AutoMode.SCORE, settings);
    scheduler.schedule(command);
    at(0.0);
    io.assertStopped();
    assertEquals(0, io.writes);
    assertFalse(command.isScheduled());
  }

  @Test
  void repeatedUseResetsClockAndState() {
    TimedAutoCommand command = auto(AutoMode.CROSS, AutoSettings.DEFAULTS);
    scheduler.schedule(command);
    at(2.0);
    assertFalse(command.isScheduled());
    time = 100.0;
    scheduler.schedule(command);
    at(100.1);
    assertEquals(0.30, io.left);
    assertEquals(0.1, command.elapsedSeconds(), 1e-9);
    at(102.0);
    assertFalse(command.isScheduled());
    io.assertStopped();
  }

  @Test
  void reservesAllMechanismsIncludingDuringWaits() {
    int[] defaultExecutions = {0};
    Command defaultDrive = Commands.run(() -> defaultExecutions[0]++, drive);
    drive.setDefaultCommand(defaultDrive);
    TimedAutoCommand command = auto(AutoMode.SCORE, AutoSettings.DEFAULTS);
    assertTrue(command.getRequirements().containsAll(java.util.Set.of(drive, intake, shooter)));
    scheduler.schedule(command);
    at(2.10);
    at(2.20);
    assertEquals(0, defaultExecutions[0]);
    assertEquals(command, drive.getCurrentCommand());
  }

  @Test
  void refreshesDriveAndMechanismsEveryTickForMotorSafety() {
    TimedAutoCommand command = auto(AutoMode.CROSS, AutoSettings.DEFAULTS);
    scheduler.schedule(command);
    for (int i = 0; i < 10; i++) { at(i * 0.02); }
    assertEquals(10, io.writes);
  }

  @Test
  void largeSchedulerDelayStopsWithoutStartingAnExpiredShot() {
    TimedAutoCommand command = auto(AutoMode.SCORE, AutoSettings.DEFAULTS);
    scheduler.schedule(command);
    at(0.0);
    time = 15.0;
    scheduler.run();
    io.assertStopped();
    assertFalse(command.isScheduled());
  }

  @Test
  void schedulerStallCannotJumpPastSpinupIntoFeed() {
    TimedAutoCommand command = auto(AutoMode.SCORE, AutoSettings.DEFAULTS);
    scheduler.schedule(command);
    at(1.0);
    assertTrue(io.left > 0.0);
    time = 3.10;
    scheduler.run();
    io.assertStopped();
    assertFalse(command.isScheduled());
    assertEquals("Loop overrun", command.phase());
  }

  @Test
  void backwardsClockStopsInsteadOfExtendingMotion() {
    TimedAutoCommand command = auto(AutoMode.CROSS, AutoSettings.DEFAULTS);
    scheduler.schedule(command);
    at(0.50);
    at(0.25);
    io.assertStopped();
    assertFalse(command.isScheduled());
    assertEquals("Clock error", command.phase());
  }
}
