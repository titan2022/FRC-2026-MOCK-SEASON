package frc.robot.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** Exercises the real DifferentialDrive path, with only the CAN endpoints replaced. */
class KitbotTankDrivetrainTest {
  private static final class FakeMotor implements MotorController {
    private boolean inverted;
    private double output;

    @Override
    public void set(double speed) {
      // Deliberately do not clamp here: out-of-range writes must fail the tests.
      output = inverted ? -speed : speed;
    }

    @Override
    public double get() {
      return output;
    }

    @Override
    public void setInverted(boolean inverted) {
      this.inverted = inverted;
    }

    @Override
    public boolean getInverted() {
      return inverted;
    }

    @Override
    public void disable() {
      output = 0.0;
    }

    @Override
    public void stopMotor() {
      output = 0.0;
    }
  }

  private FakeMotor left;
  private FakeMotor right;
  private KitbotTankDrivetrain drivetrain;

  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @BeforeEach
  void before() {
    left = new FakeMotor();
    right = new FakeMotor();
    drivetrain = new KitbotTankDrivetrain(left, right);
  }

  @AfterEach
  void after() {
    drivetrain.close();
    CommandScheduler.getInstance().unregisterSubsystem(drivetrain);
  }

  @Test
  void autoWritesActualDutyWithoutTeleopScalingOrSquaring() {
    drivetrain.driveAutonomous(0.30, -0.20);
    assertOutputs(0.30, -0.20);
  }

  @Test
  void smallAutonomousDutyBypassesJoystickDeadband() {
    drivetrain.driveAutonomous(0.04, 0.05);
    assertOutputs(0.04, 0.05);
  }

  @Test
  void driverMaximumIsRestoredAfterAutonomous() {
    drivetrain.driveAutonomous(0.85, 0.85);
    assertOutputs(0.85, 0.85);
    drivetrain.arcadeDrive(1.0, 0.0);
    assertOutputs(0.60, 0.60);
    drivetrain.arcadeDrive(-1.0, 0.0);
    assertOutputs(-0.60, -0.60);
  }

  @Test
  void bothAutoAndTeleopClampOverrangeInputsBeforeMotorWrites() {
    drivetrain.driveAutonomous(5.0, -5.0);
    assertOutputs(1.0, -1.0);
    drivetrain.arcadeDrive(5.0, 0.0);
    assertOutputs(0.60, 0.60);
    drivetrain.arcadeDrive(0.0, 3.0);
    assertTrue(Math.abs(left.get()) <= 0.60);
    assertTrue(Math.abs(right.get()) <= 0.60);
    assertEquals(-left.get(), right.get(), 1e-12);
  }

  @Test
  void invalidInputOnEitherSideStopsBothMotorsAndTelemetry() {
    for (double invalid : new double[] {
        Double.NaN, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY}) {
      drivetrain.driveAutonomous(0.30, 0.30);
      drivetrain.driveAutonomous(invalid, 0.30);
      assertOutputs(0.0, 0.0);
      drivetrain.driveAutonomous(0.30, 0.30);
      drivetrain.driveAutonomous(0.30, invalid);
      assertOutputs(0.0, 0.0);
      drivetrain.arcadeDrive(0.8, 0.2);
      drivetrain.arcadeDrive(invalid, 0.0);
      assertOutputs(0.0, 0.0);
      drivetrain.arcadeDrive(0.8, 0.2);
      drivetrain.arcadeDrive(0.8, invalid);
      assertOutputs(0.0, 0.0);
    }
  }

  @Test
  void brakeZerosRequestsAndActualWritesFromEitherDriveMode() {
    drivetrain.driveAutonomous(0.35, 0.25);
    drivetrain.brake();
    assertOutputs(0.0, 0.0);
    drivetrain.arcadeDrive(0.8, -0.2);
    drivetrain.brake();
    assertOutputs(0.0, 0.0);
  }

  @Test
  void trimKeepsLeftRightOrderAndTelemetryPrecedesMotorInversion() {
    right.setInverted(true);
    // Positive trim requests faster left and slower right on a forward run.
    drivetrain.driveAutonomous(0.34, 0.26);
    assertEquals(0.34, drivetrain.getLeftRequestedOutput(), 1e-12);
    assertEquals(0.26, drivetrain.getRightRequestedOutput(), 1e-12);
    assertEquals(0.34, left.get(), 1e-12);
    assertEquals(-0.26, right.get(), 1e-12);
    drivetrain.brake();
    assertOutputs(0.0, 0.0);
  }

  @Test
  void ordinaryJoystickDeadbandStillStopsTheDriver() {
    drivetrain.arcadeDrive(0.03, -0.04);
    assertOutputs(0.0, 0.0);
  }

  private void assertOutputs(double expectedLeft, double expectedRight) {
    assertEquals(expectedLeft, drivetrain.getLeftRequestedOutput(), 1e-12);
    assertEquals(expectedRight, drivetrain.getRightRequestedOutput(), 1e-12);
    assertEquals(expectedLeft, left.get(), 1e-12);
    assertEquals(expectedRight, right.get(), 1e-12);
  }
}
