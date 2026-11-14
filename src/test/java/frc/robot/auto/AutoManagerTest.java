package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SendableChooserSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

class AutoManagerTest {
  private static final class FakeIO implements AutoIO {
    double left, right, intake, shooter;
    public void drive(double leftDuty, double rightDuty) { left = leftDuty; right = rightDuty; }
    public void intake(double volts) { intake = volts; }
    public void shooter(double volts) { shooter = volts; }
    public void stop() { left = right = intake = shooter = 0.0; }
  }

  private NetworkTableInstance settingsInstance;
  private NetworkTable table;
  private AutoManager manager;
  private SendableChooserSim chooser;
  private FakeIO io;
  private SubsystemBase drive, intake, shooter;

  @BeforeAll
  static void initializeHal() { assertTrue(HAL.initialize(500, 0)); }

  @BeforeEach
  void before() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().clearComposedCommands();
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    CommandScheduler.getInstance().enable();
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    settingsInstance = NetworkTableInstance.create();
    settingsInstance.startLocal();
    table = settingsInstance.getTable("Settings");
    io = new FakeIO();
    drive = new SubsystemBase() {};
    intake = new SubsystemBase() {};
    shooter = new SubsystemBase() {};
    manager = new AutoManager(io, table, drive, intake, shooter);
    chooser = new SendableChooserSim("/SmartDashboard/Auto Chooser/");
    select(AutoMode.NONE.displayName());
  }

  @AfterEach
  void after() {
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().unregisterSubsystem(drive, intake, shooter);
    chooser.close();
    manager.chooser().close();
    settingsInstance.close();
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  private void select(String name) {
    chooser.setSelected(name);
    SmartDashboard.updateValues();
  }

  private void set(String key, double value) {
    assertTrue(table.getEntry(AutoSettings.PREFIX + key).setDouble(value));
  }

  @Test
  void defaultsToNoneAndUnknownSelectionFailsClosed() {
    assertEquals("Do nothing", manager.selectedName());
    assertEquals(AutoMode.NONE, ((TimedAutoCommand) manager.createSelectedCommand()).mode());
    select("stale dashboard option");
    assertEquals("Do nothing", manager.selectedName());
    assertEquals(AutoMode.NONE, ((TimedAutoCommand) manager.createSelectedCommand()).mode());
  }

  @Test
  void chooserResolvesBothRealRoutines() {
    select(AutoMode.CROSS.displayName());
    assertEquals(AutoMode.CROSS, ((TimedAutoCommand) manager.createSelectedCommand()).mode());
    select(AutoMode.SCORE.displayName());
    assertEquals(AutoMode.SCORE, ((TimedAutoCommand) manager.createSelectedCommand()).mode());
  }

  @Test
  void liveEditsDoNotChangeSnapshotAndNextRunGetsNewValues() {
    select(AutoMode.CROSS.displayName());
    TimedAutoCommand first = (TimedAutoCommand) manager.createSelectedCommand();
    CommandScheduler.getInstance().schedule(first);
    set("Cross drive power", 0.50);
    set("Cross drive seconds", 3.0);
    CommandScheduler.getInstance().run();
    assertEquals(0.30, io.left, 1e-9);
    assertEquals(2.0, manager.plannedSeconds());
    first.cancel();
    TimedAutoCommand second = (TimedAutoCommand) manager.createSelectedCommand();
    assertEquals(0.50, second.settings().crossDrivePower());
    assertEquals(3.0, second.plannedSeconds());
    assertEquals(0.30, first.settings().crossDrivePower());
  }

  @Test
  void invalidTuningIsVisibleAndBlocksAllOutputs() {
    select(AutoMode.SCORE.displayName());
    set("Shot power", Double.NaN);
    assertFalse(manager.tuningValid());
    manager.periodic();
    assertFalse(SmartDashboard.getBoolean("Auto/Tuning valid", true));
    assertTrue(SmartDashboard.getString("Auto/Tuning status", "").contains("Shot power"));
    TimedAutoCommand command = (TimedAutoCommand) manager.createSelectedCommand();
    CommandScheduler.getInstance().schedule(command);
    CommandScheduler.getInstance().run();
    assertFalse(command.isScheduled());
    assertEquals(0.0, io.left);
    assertEquals(0.0, io.intake);
    assertEquals(0.0, io.shooter);
    assertTrue(manager.status().startsWith("Blocked:"));
  }

  @Test
  void wrongTypedTuningDoesNotSilentlyUseDefault() {
    // Construct a malformed retained topic before the manager installs defaults.
    // NT deliberately forbids changing an already-subscribed double topic to a string.
    manager.chooser().close();
    settingsInstance.close();
    settingsInstance = NetworkTableInstance.create();
    settingsInstance.startLocal();
    table = settingsInstance.getTable("MalformedSettings");
    assertTrue(table.getEntry(AutoSettings.PREFIX + "Cross drive power").setString("fast"));
    manager = new AutoManager(io, table, drive, intake, shooter);
    select(AutoMode.CROSS.displayName());
    assertFalse(manager.tuningValid());
    TimedAutoCommand command = (TimedAutoCommand) manager.createSelectedCommand();
    assertTrue(Double.isNaN(command.settings().crossDrivePower()));
    CommandScheduler.getInstance().schedule(command);
    CommandScheduler.getInstance().run();
    assertEquals("Invalid settings", manager.phase());
    assertEquals(0.0, io.left);
  }

  @Test
  void autoShotPowerRemainsIndependentOfNextRunTuning() {
    select(AutoMode.SCORE.displayName());
    set("Shot power", 0.80);
    manager.createSelectedCommand();
    set("Shot power", 0.60);
    assertEquals(0.80, manager.shotPower());
    assertFalse(manager.shotActive());
  }
}
