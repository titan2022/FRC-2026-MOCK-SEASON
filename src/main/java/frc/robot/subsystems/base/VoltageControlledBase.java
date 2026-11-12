package frc.robot.subsystems.base;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;

import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Base for all subsystems which are a single mechanism controlled by constant
 * voltage.
 * This is simpler than PIDF control and is suitable for mechanisms like
 * indexers.
 */
public class VoltageControlledBase extends SubsystemBase {
  // The following fields must be defined by subclasses.
  public String SUBSYSTEM_NAME = "WARNING NAME NOT SET";

  // Hardware devices
  public WPI_TalonSRX motor;

  // Configuration
  public TalonSRXConfiguration motorConfig;

  // Default voltage for operation
  public double DEFAULT_VOLTAGE = 6.0;

  // Motor controller request
  protected VoltageOut voltageRequest = new VoltageOut(0);

  // Motor controller signals
  protected StatusSignal<Angle> positionSignal;
  protected StatusSignal<AngularVelocity> velocitySignal;
  protected StatusSignal<Voltage> voltageSignal;
  protected StatusSignal<Current> statorCurrentSignal;
  protected StatusSignal<Temperature> temperatureSignal;

  // Current setpoint
  protected double voltageSetpoint = 0;

  // Tunables
  // These fields are initialized in initialize()
  protected DoubleSubscriber setpoint_subscriber;

  /**
   * Creates a new Voltage Controlled Subsystem.
   */
  public VoltageControlledBase() {
  }

  // This method MUST be called at the end of subsystem initializers
  protected void initialize() {
    // Get status signals
    motor.configAllSettings(motorConfig);

    setpoint_subscriber = DogLog.tunable(
      SUBSYSTEM_NAME + "/voltage setpoint",
      voltageSetpoint,
      this::setVoltage);
  }

  /**
   * Update telemetry.
   */
  @Override
  public void periodic() {

    // Log values
    // btw prob not using these in Mock season
    DogLog.log(SUBSYSTEM_NAME + "/Voltage", getVoltage());
    DogLog.log(SUBSYSTEM_NAME + "/Stator Current", getCurrent());
    DogLog.log(SUBSYSTEM_NAME + "/Temperature", getTemperature());
    // DogLog.log(SUBSYSTEM_NAME + "/Voltage Setpoint", getVoltageSetpoint());
  }

  /**
   * Get the current applied voltage.
   * 
   * @return Applied voltage
   */
  public double getVoltage() {
    return motor.getMotorOutputVoltage();
  }

  /**
   * Get the current motor current.
   * 
   * @return Motor current in amps
   */
  public double getCurrent() {
    return motor.getOutputCurrent();
  }

  /**
   * Get the current motor temperature.
   * 
   * @return Motor temperature in Celsius
   */
  public double getTemperature() {
    return motor.getTemperature();
  }

  /**
   * Get the current voltage setpoint.
   * 
   * @return Current voltage setpoint
   */
  public double getVoltageSetpoint() {
    return voltageSetpoint;
  }

  /**
   * Set motor voltage.
   * 
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    voltageSetpoint = voltage;
    motor.setVoltage(voltage);
  }

  /**
   * Run at default voltage.
   */
  public void forward() {
    setVoltage(DEFAULT_VOLTAGE);
  }

  /**
   * Run in reverse at default voltage.
   */
  public void reverse() {
    setVoltage(-DEFAULT_VOLTAGE);
  }

  /**
   * Stop the motor.
   */
  public void stop() {
    setVoltage(0);
    motor.stopMotor();
  }

  /**
   * Creates a command to run at a specific voltage.
   * 
   * @param voltage The voltage to apply
   * @return A command that runs the mechanism at the specified voltage
   */
  public Command setVoltageCommand(double voltage) {
    return runEnd(() -> setVoltage(voltage), this::stop);
  }

  /**
   * Creates a command to run at default voltage.
   * 
   * @return A command that runs the mechanism
   */
  public Command forwardCommand() {
    return runEnd(this::forward, this::stop);
  }

  /**
   * Creates a command to run in reverse.
   * 
   * @return A command that reverses the mechanism
   */
  public Command reverseCommand() {
    return runEnd(this::reverse, this::stop);
  }

  /**
   * Creates a command to stop the mechanism.
   * 
   * @return A command that stops the mechanism
   */
  public Command stopCommand() {
    return runOnce(this::stop);
  }
}