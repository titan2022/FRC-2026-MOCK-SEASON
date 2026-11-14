# Driver dashboard

Open `elastic-layout.json` in **Elastic 2026** using **File > Open Layout** (`Ctrl+O`).
Connect Elastic to the same robot as the Driver Station. Set the driver gamepad
to USB 0 and the operator gamepad to USB 1 in the Driver Station.

The layout consumes the robot's NetworkTables data. It does not enable the robot,
write motor outputs, or require a new vendor library. The robot publishes the
driver values under `/SmartDashboard/Driver/` at up to 10 Hz. Ordinary
SmartDashboard can also display those values if Elastic is unavailable.

## Driver tab

The main screen fits a 1280 × 512 grid, excluding Elastic's own toolbar and tabs.
Lock the layout with **Edit > Lock Layout** after opening it so a mouse movement
does not accidentally move a widget during the match.

| Display | Meaning |
| --- | --- |
| Next teleop shot / XYAB | The operator's current selection: X 80%, Y 90%, A 100%, B 60%. Startup is A 100%. |
| Current shot / latched power | Power captured when the current shot began. Changing XYAB during a shot changes the next teleop shot, not the shot already underway. Idle clears this display. |
| Shot phase | The current timed reverse, spinup, or feeding phase. It is not an RPM measurement or a ball sensor. |
| Auto / next enable | Do nothing, Cross line, or Cross and score. The robot captures the selection when autonomous starts. |
| Auto this run | The routine captured for this run; it stays visible after completion. |
| Auto phase / status / timing | Current phase, result or validation error, and elapsed/planned seconds for this run. |
| Auto valid | The next selected routine's settings pass software validation. This does not mean the path has been calibrated on the field. |
| Drive / intake / shooter | Requested logical motor outputs. Positive and negative indicate forward/reverse commands; these are not measured wheel speed or shooter RPM. |
| Battery / brownout / controllers | Robot voltage, brownout flag, and whether the expected controller ports are present. |
| Match clock | Driver Station time, or `Unavailable` when the Driver Station does not provide a match clock. |
| Attention | Current connection, controller, battery, brownout, emergency-stop, or auto-configuration issues. |

Autonomous shot power is separate from XYAB. Configure it on **Auto setup**.
The current-shot widgets also show the latched autonomous shot power while the
autonomous routine is shooting.

## Auto setup tab

Connect to the robot before editing. Each number has a submit button. Enter the
number, submit it, then check the validation status at the top. Settings are
captured at autonomous start; edits made during a run apply to the next run.
Settings persist across robot restarts.

Drive powers are signed duty fractions between -0.60 and 0.60, before motor
inversion. The trim can reduce the usable range further because neither side
may exceed 0.60. Shot power is 0 to 1; durations are seconds and feed voltage is
volts. A nonzero trim is an open-loop correction and needs a field check.

The defaults are starting values. Measure the crossing and scoring-approach
durations on the actual field, with the intended starting position and direction.
The interface does not infer distance or aim from camera images.

Every tuning widget declares `data_type: "double"`. The robot announces these
entries as doubles. Elastic's numeric editor uses the announced topic type to
parse a submitted value, so a whole-number input such as `2` is still transmitted
as a double. If the topic has not been announced because the robot is disconnected,
connect before submitting a value. The parser uses
[`TextDisplayModel.publishData`](https://github.com/Gold872/elastic_dashboard/blob/v2026.0.0/lib/widgets/nt_widgets/single_topic/text_display.dart)
and [`NT4Type.convertString`](https://github.com/Gold872/elastic_dashboard/blob/v2026.0.0/lib/services/nt4_type.dart).

## Diagnostics and Cameras

**Diagnostics** contains the Driver Station connection, emergency-stop flag,
low-battery flag, selected auto, mechanism directions, and a telemetry update
counter. The counter helps distinguish a changing stream from cached values;
check Elastic's connection indicator as well.

**Cameras** contains two optional CameraServer topics named `USB Camera 0` and
`USB Camera 1`. The robot code does not start camera capture. If the robot already
publishes a stream with a different name, drag its `CameraPublisher` entry into
this tab. Empty camera widgets do not affect the driver screen or autonomous
routines.

## Checks

`python dashboard/validate_layout.py` checks the Elastic JSON structure used here,
grid bounds, overlapping widgets, every tuning field, and topic names against
the actual Java publishers. It also checks drive-power labels against
`AutoSettings.MAX_DRIVE_POWER` and requires double-type tuning editors.
Java tests exercise selection, latched-shot display, clearing, connection states,
clock handling, and the 10 Hz update behavior.

The JSON format and widget names were checked against Elastic's
[2026.0.0 test layout](https://github.com/Gold872/elastic_dashboard/blob/v2026.0.0/test_resources/test-layout.json)
and widget parsers. Actual visual inspection is still needed on the driver
laptop. No live robot connection or field run is claimed by the layout checks.

Reference: [Elastic layout import instructions](https://frc-elastic.gitbook.io/docs/getting-started/app-navigation).
