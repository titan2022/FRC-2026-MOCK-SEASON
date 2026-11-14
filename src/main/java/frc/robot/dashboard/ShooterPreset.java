package frc.robot.dashboard;

/** The operator's existing XYAB presets; these select the next teleop shot. */
public enum ShooterPreset {
  X("X", 0.80),
  Y("Y", 0.90),
  A("A", 1.00),
  B("B", 0.60);

  private final String button;
  private final double power;

  ShooterPreset(String button, double power) {
    this.button = button;
    this.power = power;
  }

  public String buttonLabel() {
    return button;
  }

  public double power() {
    return power;
  }

  public int percentage() {
    return (int) Math.round(power * 100);
  }

  public String label() {
    return button + " " + percentage() + "%";
  }
}
