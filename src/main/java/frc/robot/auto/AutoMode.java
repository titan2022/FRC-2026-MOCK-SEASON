package frc.robot.auto;

/** The two timed routines and a safe default. */
public enum AutoMode {
  NONE("Do nothing"),
  CROSS("Cross line"),
  SCORE("Cross and score");

  private final String displayName;

  AutoMode(String displayName) {
    this.displayName = displayName;
  }

  public String displayName() {
    return displayName;
  }
}
