package frc.robot.auto;

import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.drive.KitbotTankDrivetrain;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/** Chooser, preflight validation, and one immutable settings snapshot per auto start. */
public final class AutoManager {
  private final SendableChooser<AutoMode> chooser = new SendableChooser<>();
  private final NetworkTable dashboard;
  private final AutoIO io;
  private final Subsystem[] requirements;
  private TimedAutoCommand active;

  public AutoManager(KitbotTankDrivetrain drivetrain, IntakeSubsystem intake,
      ShooterSubsystem shooter) {
    this(new AutoIO() {
      @Override
      public void drive(double left, double right) { drivetrain.driveAutonomous(left, right); }
      @Override
      public void intake(double volts) { intake.setVoltage(volts); }
      @Override
      public void shooter(double volts) { shooter.setVoltage(volts); }
      @Override
      public void stop() {
        drivetrain.brake();
        intake.stop();
        shooter.stop();
      }
    }, drivetrain, intake, shooter);
  }

  AutoManager(AutoIO io, Subsystem... requirements) {
    this(io, NetworkTableInstance.getDefault().getTable("SmartDashboard"), requirements);
  }

  /** Isolated settings table for tests; production uses SmartDashboard on the default instance. */
  AutoManager(AutoIO io, NetworkTable settingsTable, Subsystem... requirements) {
    this.io = io;
    this.dashboard = settingsTable;
    this.requirements = requirements.clone();
    chooser.setDefaultOption(AutoMode.NONE.displayName(), AutoMode.NONE);
    chooser.addOption(AutoMode.CROSS.displayName(), AutoMode.CROSS);
    chooser.addOption(AutoMode.SCORE.displayName(), AutoMode.SCORE);
    SmartDashboard.putData("Auto Chooser", chooser);
    AutoSettings.defaultValues().forEach((name, value) -> {
      dashboard.getEntry(AutoSettings.PREFIX + name).setDefaultDouble(value);
      dashboard.getEntry(AutoSettings.PREFIX + name).setPersistent();
    });
  }

  public SendableChooser<AutoMode> chooser() { return chooser; }

  private AutoMode selected() {
    AutoMode selected = chooser.getSelected();
    return selected == null ? AutoMode.NONE : selected;
  }

  private AutoSettings readSettings() {
    return AutoSettings.read(name -> {
      NetworkTableValue value = dashboard.getEntry(AutoSettings.PREFIX + name).getValue();
      // A string, boolean, or missing entry is an invalid setting, not a silent fallback.
      return value.isDouble() ? value.getDouble() : Double.NaN;
    });
  }

  /** Called by autonomousInit. Dashboard edits after this call apply only to the next run. */
  public Command createSelectedCommand() {
    active = new TimedAutoCommand(selected(), readSettings(), io, requirements);
    return active;
  }

  public String selectedName() { return selected().displayName(); }
  public String activeName() { return active == null ? "None" : active.mode().displayName(); }
  public String phase() { return active == null ? "Idle" : active.phase(); }

  public String status() {
    if (active != null) {
      return active.status();
    }
    List<String> errors = readSettings().errors(selected());
    return errors.isEmpty() ? "Ready; drive timing needs field calibration"
        : "Blocked: " + String.join("; ", errors);
  }

  public double elapsedSeconds() { return active == null ? 0.0 : active.elapsedSeconds(); }

  public double plannedSeconds() {
    if (active != null) {
      return active.plannedSeconds();
    }
    AutoSettings preview = readSettings();
    return preview.errors(selected()).isEmpty() ? preview.totalSeconds(selected()) : 0.0;
  }

  public boolean tuningValid() { return readSettings().errors(selected()).isEmpty(); }

  public boolean shotActive() {
    if (active == null || !active.isScheduled() || active.mode() != AutoMode.SCORE) {
      return false;
    }
    return switch (active.phase()) {
      case "Reversing ball", "Shooter spin-up delay", "Feeding shot" -> true;
      default -> false;
    };
  }

  /** Latched power for this run; independent of the operator's XYAB selection. */
  public double shotPower() { return active == null ? 0.0 : active.settings().shotPower(); }

  public void periodic() {
    // Keep a plain SmartDashboard preflight message available without the custom dashboard.
    List<String> errors = readSettings().errors(selected());
    SmartDashboard.putString("Auto/Tuning status", errors.isEmpty()
        ? "Valid for next run; changes are captured at autonomous start"
        : String.join("; ", errors));
    SmartDashboard.putBoolean("Auto/Tuning valid", errors.isEmpty());
    SmartDashboard.putNumber("Auto/Next run seconds", errors.isEmpty()
        ? readSettings().totalSeconds(selected()) : 0.0);
  }
}
