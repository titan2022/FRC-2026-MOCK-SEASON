# Fall 2026 Mock Season

Titan Robotics code for the 2026 Gear Up Games, preparing for the 2027 FRC season.
The project uses WPILib / GradleRIO 2026.2.1 and Java 17.

## Autonomous

The dashboard chooser offers:

- **Do nothing**: the startup default; stop all motors.
- **Cross line**: optional start delay, timed straight drive, stop.
- **Cross and score**: timed approach, settle, reverse the ball, spin up the shooter,
  feed one preload, optional exit, stop.

The robot has no configured drive encoder, gyro, or ball sensor. The drive times
are initial tuning values. Measure them on the field from a repeatable starting
mark. The scoring approach must move all bumpers into the scoring zone; the
shorter crossing-only route may not do that.

Use [the auto tuning guide](docs/AUTO_TUNING.md) for settings, first-run procedure,
Practice Mode checks, and fallback choices. Game rules are in the
[official manual](https://fallgame.gearitforward.com/2026/).

## Driver dashboard

Open [dashboard/elastic-layout.json](dashboard/elastic-layout.json) in Elastic 2026.
The Driver tab shows the selected XYAB preset, the current shot's latched power,
shot phase, autonomous selection and progress, motor requests, match clock,
battery, and controller presence. Other tabs contain auto tuning, diagnostics,
and optional camera widgets.

[Dashboard instructions](dashboard/README.md) explain connection, tuning inputs,
and the meaning of each display. The same telemetry is available under
`/SmartDashboard/Driver/` in other NetworkTables dashboards.

## Controls

| Input | Action |
| --- | --- |
| Driver left stick Y | Arcade forward/backward, maximum 60% output |
| Driver left X + driver right X + operator left X | Combined arcade steering |
| Operator left bumper | Capture: forward intake, reverse shooter |
| Operator right bumper | Reverse intake; overrides capture and interrupts a shot |
| Operator X / Y / A / B | Select 80% / 90% / 100% / 60% for the next teleop shot |
| Operator right trigger | One timed shot per fresh press |

Driver controller is USB 0; operator controller is USB 1. Startup shot preset is
A / 100%. A shot captures its selected power when it begins. Changing XYAB while
it runs changes the next teleop shot. Autonomous shot power is a separate setting.

## Hardware assignments

| Mechanism | CAN ID | Inverted | Neutral mode |
| --- | ---: | --- | --- |
| Left drivetrain | 13 | No | Brake |
| Right drivetrain | 15 | Yes | Brake |
| Intake | 3 | No | Brake |
| Shooter | 2 | No | Coast |

All four controllers are Talon SRX. These values describe this source version;
verify that they match the robot before running it.

## Build and test

Use a WPILib 2026 terminal with its Java 17 JDK:

```bash
./gradlew build
python3 dashboard/validate_layout.py
```

Windows uses `gradlew.bat build` and `python dashboard/validate_layout.py`.
The first build downloads WPILib and vendor dependencies when they are not
already installed.

The tests exercise actual WPILib command scheduling, Driver Station simulation,
NetworkTables selection and snapshots, drivetrain output calculations, shot
sequencing, control bindings, mode changes, and dashboard values. They do not
establish real travel distance or shooting accuracy.

## Implementation map

| Location | Responsibility |
| --- | --- |
| `src/main/java/frc/robot/auto/AutoSettings.java` | Source defaults, units, bounds, and duration validation |
| `src/main/java/frc/robot/auto/AutoManager.java` | Chooser, persistent tuning entries, captured run settings |
| `src/main/java/frc/robot/auto/TimedAutoCommand.java` | Timed phases, motor requests, interruption cleanup |
| `src/main/java/frc/robot/dashboard/DriverDashboard.java` | Read-only driver telemetry, published at up to 10 Hz |
| `src/main/java/frc/robot/dashboard/ShooterPreset.java` | XYAB mapping and display labels |
| `src/main/java/frc/robot/Robot.java` | Robot lifecycle, controls, autonomous and dashboard integration |
| `src/test/java/frc/robot/` | Configuration, command, drivetrain, dashboard, and whole-robot tests |
