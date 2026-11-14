package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.base.VoltageControlledBase;

class ShootCommandTest {
  private static class FakeRoller extends VoltageControlledBase {
    FakeRoller() {
      DEFAULT_VOLTAGE = 12.0;
    }

    @Override
    public void setVoltage(double volts) {
      voltageSetpoint = volts;
    }

    @Override
    public void stop() {
      voltageSetpoint = 0.0;
    }

    @Override
    public void periodic() {
      // No CAN devices in this test.
    }
  }

  private final CommandScheduler scheduler = CommandScheduler.getInstance();
  private FakeRoller intake;
  private FakeRoller shooter;
  private double selectedPower;
  private ShootCommand command;

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @BeforeEach
  void before() {
    scheduler.cancelAll();
    scheduler.clearComposedCommands();
    scheduler.getActiveButtonLoop().clear();
    scheduler.enable();
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    intake = new FakeRoller();
    shooter = new FakeRoller();
    selectedPower = 0.8;
    command = new ShootCommand(intake, shooter, () -> selectedPower);
  }

  @AfterEach
  void after() {
    scheduler.cancelAll();
    scheduler.unregisterSubsystem(intake, shooter);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
  }

  private void tick() {
    SimHooks.stepTiming(0.02);
    scheduler.run();
  }

  private void runFor(double seconds) {
    for (int i = 0; i < Math.ceil(seconds / 0.02); i++) {
      tick();
    }
  }

  @Test
  void reversesThenSpinsBeforeFeedingAndStopsAtEnd() {
    command.schedule();
    boolean sawReverse = false;
    boolean sawSpinup = false;
    boolean sawFeed = false;
    int spinupTicks = 0;
    for (int i = 0; i < 125 && command.isScheduled(); i++) {
      tick();
      if (intake.getVoltageSetpoint() < 0) {
        assertFalse(sawSpinup);
        assertFalse(sawFeed);
        assertEquals(-12.0, shooter.getVoltageSetpoint());
        sawReverse = true;
      }
      if (shooter.getVoltageSetpoint() > 0 && intake.getVoltageSetpoint() == 0) {
        assertTrue(sawReverse);
        sawSpinup = true;
        spinupTicks++;
      }
      if (intake.getVoltageSetpoint() > 0) {
        assertTrue(sawSpinup);
        assertTrue(spinupTicks >= 25, "Feed must wait through the shooter spin-up interval");
        assertEquals(Constants.FEED_VOLTAGE, intake.getVoltageSetpoint());
        assertEquals(9.6, shooter.getVoltageSetpoint(), 1e-9);
        sawFeed = true;
      }
    }
    assertTrue(sawReverse && sawSpinup && sawFeed);
    assertFalse(command.isScheduled());
    assertEquals(0.0, intake.getVoltageSetpoint());
    assertEquals(0.0, shooter.getVoltageSetpoint());
    assertEquals("Complete", command.getPhase());
  }

  @Test
  void presetChangesApplyToNextShotOnly() {
    command.schedule();
    runFor(0.3);
    selectedPower = 0.6;
    runFor(0.2);
    assertEquals(0.8, command.getPower());
    assertEquals(9.6, shooter.getVoltageSetpoint(), 1e-9);
    runFor(1.8);
    assertFalse(command.isScheduled());
    command.schedule();
    runFor(0.5);
    assertEquals(0.6, command.getPower());
    assertEquals(7.2, shooter.getVoltageSetpoint(), 1e-9);
  }

  @ParameterizedTest
  @ValueSource(doubles = {0.04, 0.12, 0.40, 1.20})
  void cancelStopsBothRollersInEveryPhase(double seconds) {
    command.schedule();
    runFor(seconds);
    assertTrue(command.isScheduled());
    command.cancel();
    assertEquals(0.0, intake.getVoltageSetpoint());
    assertEquals(0.0, shooter.getVoltageSetpoint());
    assertEquals("Interrupted", command.getPhase());
  }

  @Test
  void disableStopsBothRollersDuringFeed() {
    command.schedule();
    runFor(1.2);
    assertEquals(Constants.FEED_VOLTAGE, intake.getVoltageSetpoint());
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    tick();
    assertFalse(command.isScheduled());
    assertEquals(0.0, intake.getVoltageSetpoint());
    assertEquals(0.0, shooter.getVoltageSetpoint());
  }

  @Test
  void holdsBothSubsystemRequirementsForEntireShot() {
    assertTrue(command.getRequirements().contains(intake));
    assertTrue(command.getRequirements().contains(shooter));
  }
}
