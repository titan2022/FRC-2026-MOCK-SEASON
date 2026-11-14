package frc.robot.integration;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.Timeout;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SendableChooserSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Robot;
import frc.robot.auto.AutoMode;
import frc.robot.auto.AutoSettings;
import frc.robot.commands.ShootCommand;

/**
 * Actual Robot bindings, scheduler, NetworkTables, and CTRE desktop simulation.
 * The test advances FPGA time explicitly; no robot connection or wall-clock sleeps.
 * These checks observe requested outputs, not physical motion or ball behavior.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
@Timeout(20)
class RobotIntegrationTest {
  private final CommandScheduler scheduler = CommandScheduler.getInstance();
  private final AtomicInteger shotStarts = new AtomicInteger();
  private Robot robot;
  private XboxControllerSim driver;
  private XboxControllerSim operator;
  private SendableChooserSim chooser;
  private NetworkTable table;

  @BeforeAll
  @Timeout(30)
  void createRobotOnce() {
    assertTrue(HAL.initialize(500, 0));
    scheduler.cancelAll();
    scheduler.getActiveButtonLoop().clear();
    scheduler.clearComposedCommands();
    scheduler.enable();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(false);
    driver = new XboxControllerSim(0);
    operator = new XboxControllerSim(1);
    resetController(driver);
    resetController(operator);
    DriverStationSim.notifyNewData();
    RoboRioSim.setVInVoltage(12.0);
    robot = new Robot();
    SimHooks.pauseTiming();
    table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    chooser = new SendableChooserSim("/SmartDashboard/Auto Chooser/");
    scheduler.onCommandInitialize(command -> {
      if (command instanceof ShootCommand) {
        shotStarts.incrementAndGet();
      }
    });
  }

  @BeforeEach
  void resetBetweenTests() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setDsAttached(true);
    resetController(driver);
    resetController(operator);
    DriverStationSim.notifyNewData();
    robot.disabledInit();
    scheduler.enable();
    AutoSettings.defaultValues().forEach((key, value) -> {
      NetworkTableEntry entry = table.getEntry(AutoSettings.PREFIX + key);
      entry.clearPersistent();
      entry.unpublish();
      assertTrue(entry.setDouble(value));
    });
    choose(AutoMode.NONE);
    tick(6); // Poll released controls while disabled to reset every real Trigger edge.
    shotStarts.set(0);
  }

  @AfterEach
  void disableAfterTest() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    robot.disabledInit();
    assertStopped();
  }

  @AfterAll
  void closeRobot() {
    scheduler.cancelAll();
    scheduler.getActiveButtonLoop().clear();
    if (chooser != null) {
      chooser.close();
    }
    if (robot != null) {
      scheduler.unregisterSubsystem(robot.drivetrain, robot.intake, robot.shooter);
      robot.autoChooser.close();
      robot.intake.motor.close();
      robot.shooter.motor.close();
      robot.drivetrain.close();
      robot.close();
    }
    SimHooks.resumeTiming();
  }

  @Test
  void holdingTriggerAcrossDisabledToTeleopDoesNotCreateAShot() {
    operator.setRightTriggerAxis(1.0);
    tick(6);
    enableTeleop();
    tick(120);
    assertEquals(0, shotStarts.get());
    assertFalse(flag("Driver/Shot active"));
    assertStopped();

    operator.setRightTriggerAxis(0.0);
    tick(6);
    operator.setRightTriggerAxis(1.0);
    tick(6);
    assertEquals(1, shotStarts.get());
    assertTrue(flag("Driver/Shot active"));
  }

  @Test
  void freshPressFiresOnceAndPresetChangesOnlyTheNextShot() {
    enableTeleop();
    operator.setBButton(true);
    tick(6);
    operator.setBButton(false);
    tick(1);
    assertEquals("B 60%", text("Driver/Preset"));

    operator.setRightTriggerAxis(1.0);
    tick(6);
    assertEquals(1, shotStarts.get());
    assertTrue(flag("Driver/Shot active"));
    assertEquals(60.0, number("Driver/Active shot percent"), 1e-9);

    operator.setXButton(true);
    tick(6);
    operator.setXButton(false);
    tick(6);
    assertEquals("X 80%", text("Driver/Preset"));
    assertEquals(60.0, number("Driver/Active shot percent"), 1e-9);
    tick(30);
    assertEquals(7.2, robot.shooter.getVoltageSetpoint(), 1e-9);

    tick(150); // Hold the trigger well beyond completion: no second shot.
    assertEquals(1, shotStarts.get());
    assertFalse(flag("Driver/Shot active"));
    assertStopped();

    operator.setRightTriggerAxis(0.0);
    tick(6);
    operator.setRightTriggerAxis(1.0);
    tick(6);
    assertEquals(2, shotStarts.get());
    assertEquals(80.0, number("Driver/Active shot percent"), 1e-9);
  }

  @Test
  void outtakeInterruptsTheShotAndDoesNotRestartWhileTriggerStaysHeld() {
    enableTeleop();
    operator.setRightTriggerAxis(1.0);
    tick(50);
    assertEquals(1, shotStarts.get());
    assertTrue(flag("Driver/Shot active"));
    assertTrue(robot.intake.getVoltageSetpoint() > 0.0);

    operator.setRightBumperButton(true);
    tick(6);
    assertFalse(flag("Driver/Shot active"));
    assertEquals(-12.0, robot.intake.getVoltageSetpoint(), 1e-9);
    assertEquals(0.0, robot.shooter.getVoltageSetpoint(), 1e-9);
    operator.setRightBumperButton(false);
    tick(30);
    assertEquals(1, shotStarts.get());
    assertStopped();
  }

  @Test
  void autonomousIgnoresOperatorControlsAndKeepsItsCapturedSettings() {
    choose(AutoMode.CROSS);
    tune("Cross drive seconds", 1.0);
    tune("Cross drive power", 0.30);
    tune("Drive trim", 0.04);
    driver.setLeftY(-1.0);
    driver.setLeftX(1.0);
    driver.setRightX(1.0);
    operator.setLeftX(1.0);
    operator.setLeftBumperButton(true);
    operator.setRightTriggerAxis(1.0);
    enableAuto();
    tick(6);
    assertEquals(0.34, robot.drivetrain.getLeftRequestedOutput(), 1e-9);
    assertEquals(0.26, robot.drivetrain.getRightRequestedOutput(), 1e-9);
    assertEquals(0.0, robot.intake.getVoltageSetpoint(), 1e-9);
    assertEquals(0.0, robot.shooter.getVoltageSetpoint(), 1e-9);
    assertEquals(0, shotStarts.get());

    tune("Cross drive power", 0.50);
    tick(6);
    assertEquals(0.34, robot.drivetrain.getLeftRequestedOutput(), 1e-9);
    assertEquals(0.26, robot.drivetrain.getRightRequestedOutput(), 1e-9);

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    robot.disabledInit();
    assertStopped();
    tick(6);
    assertEquals("Interrupted", text("Driver/Auto phase"));
    assertEquals(0, shotStarts.get());
  }

  @Test
  void autoShotTelemetryUsesItsOwnPowerAndDisableStopsBothMechanisms() {
    enableTeleop();
    operator.setAButton(true);
    tick(6);
    operator.setAButton(false);
    tick(6);
    assertEquals("A 100%", text("Driver/Preset"));

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    robot.disabledInit();
    choose(AutoMode.SCORE);
    tune("Score approach seconds", 0.20);
    tune("Score settle seconds", 0.10);
    tune("Shot reverse seconds", 0.10);
    tune("Shot spinup seconds", 0.30);
    tune("Shot feed seconds", 0.60);
    tune("Shot power", 0.60);
    enableAuto();
    tick(45);
    assertEquals("Feeding shot", text("Driver/Auto phase"));
    assertTrue(flag("Driver/Shot active"));
    assertEquals(60.0, number("Driver/Active shot percent"), 1e-9);
    assertEquals("A 100%", text("Driver/Preset"));
    assertEquals(7.2, robot.shooter.getVoltageSetpoint(), 1e-9);
    assertEquals(6.0, robot.intake.getVoltageSetpoint(), 1e-9);
    assertEquals(0, shotStarts.get());

    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    robot.disabledInit();
    assertStopped();
    tick(6);
    assertFalse(flag("Driver/Shot active"));
  }

  @Test
  void switchingFromAutoToTeleopRestoresDriverAndItsSixtyPercentLimit() {
    choose(AutoMode.CROSS);
    tune("Cross drive seconds", 1.0);
    tune("Cross drive power", 0.30);
    driver.setLeftY(-1.0);
    enableAuto();
    tick(6);
    assertEquals(0.30, robot.drivetrain.getLeftRequestedOutput(), 1e-9);
    assertEquals(0.30, robot.drivetrain.getRightRequestedOutput(), 1e-9);

    enableTeleop();
    tick(6);
    assertEquals(0.60, robot.drivetrain.getLeftRequestedOutput(), 1e-9);
    assertEquals(0.60, robot.drivetrain.getRightRequestedOutput(), 1e-9);
    assertEquals(0.0, robot.intake.getVoltageSetpoint(), 1e-9);
    assertEquals(0.0, robot.shooter.getVoltageSetpoint(), 1e-9);
    assertEquals("Interrupted", text("Driver/Auto phase"));
  }

  @Test
  void finishedAutoRemainsStoppedWithJoysticksAndCaptureHeld() {
    choose(AutoMode.CROSS);
    tune("Cross drive seconds", 0.20);
    driver.setLeftY(-1.0);
    driver.setLeftX(1.0);
    operator.setLeftX(1.0);
    operator.setLeftBumperButton(true);
    operator.setRightTriggerAxis(1.0);
    enableAuto();
    tick(100);
    assertEquals("Complete", text("Driver/Auto phase"));
    assertEquals(0, shotStarts.get());
    assertStopped();
    tick(100);
    assertStopped();
  }

  private void enableTeleop() {
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    robot.teleopInit();
  }

  private void enableAuto() {
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    robot.autonomousInit();
  }

  private void choose(AutoMode mode) {
    chooser.setSelected(mode.displayName());
    SmartDashboard.updateValues();
  }

  private void tune(String key, double value) {
    assertTrue(table.getEntry(AutoSettings.PREFIX + key).setDouble(value));
  }

  private void tick(int count) {
    for (int i = 0; i < count; i++) {
      DriverStationSim.notifyNewData();
      // This fixture calls robotPeriodic directly rather than starting TimedRobot's
      // notifier thread. Do not wait for that unstarted notifier when stepping time.
      SimHooks.stepTimingAsync(0.02);
      robot.robotPeriodic();
      SmartDashboard.updateValues();
    }
  }

  private static void resetController(XboxControllerSim controller) {
    controller.setLeftX(0.0);
    controller.setLeftY(0.0);
    controller.setRightX(0.0);
    controller.setRightY(0.0);
    controller.setLeftTriggerAxis(0.0);
    controller.setRightTriggerAxis(0.0);
    controller.setAButton(false);
    controller.setBButton(false);
    controller.setXButton(false);
    controller.setYButton(false);
    controller.setLeftBumperButton(false);
    controller.setRightBumperButton(false);
    controller.setBackButton(false);
    controller.setStartButton(false);
    controller.setLeftStickButton(false);
    controller.setRightStickButton(false);
    controller.setPOV(-1);
  }

  private boolean flag(String key) {
    return table.getEntry(key).getBoolean(false);
  }

  private String text(String key) {
    return table.getEntry(key).getString("");
  }

  private double number(String key) {
    return table.getEntry(key).getDouble(Double.NaN);
  }

  private void assertStopped() {
    assertEquals(0.0, robot.drivetrain.getLeftRequestedOutput(), 1e-9);
    assertEquals(0.0, robot.drivetrain.getRightRequestedOutput(), 1e-9);
    assertEquals(0.0, robot.intake.getVoltageSetpoint(), 1e-9);
    assertEquals(0.0, robot.shooter.getVoltageSetpoint(), 1e-9);
  }
}
