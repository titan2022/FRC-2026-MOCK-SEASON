# Auto tuning for mock season

The code provides **Do nothing**, **Cross line**, and **Cross and score**.
Do nothing is the default. The other routines use time and motor output.
The robot has no configured encoder, gyro, or ball sensor.
The default drive times are starting values, not measured field distances.

The [game manual](https://fallgame.gearitforward.com/2026/) gives 10 points for entering the scoring zone and 10 points for an autonomous goal.
Any part of the robot can earn the crossing points.
All bumpers must enter the scoring zone before the robot ejects a ball.
The robot must remain in its own alliance half during auto.

## Routines

| Choice | Sequence | Default duration |
| --- | --- | ---: |
| Do nothing | Stop all outputs | 0 s |
| Cross line | Start delay, straight drive, stop | 2.00 s |
| Cross and score | Start delay, approach, settle, reverse ball, spin shooter, feed ball, optional exit, stop | 4.02 s |

Cross and score preserves the current shot sequence.
Both rollers reverse at -12 V during the reverse phase.
The shooter then runs alone during the spinup phase.
The intake feeds during the final shot phase.
The exit phase has zero duration by default.

Auto shot power is separate from the operator's XYAB selection.
XYAB controls the next teleop shot: X 80%, Y 90%, A 100%, and B 60%.

## Settings

The Elastic layout has an **Auto setup** tab.
The same entries exist under `/SmartDashboard/Auto/Tuning/`.
The table gives the exact entry names after that prefix.

| Entry | Default | Unit and meaning |
| --- | ---: | --- |
| Start delay seconds | 0.00 | Seconds with all outputs stopped before either routine |
| Cross drive seconds | 2.00 | Seconds of drive in Cross line |
| Cross drive power | 0.30 | Signed motor duty for Cross line |
| Drive trim | 0.00 | Add to left duty and subtract from right duty during each drive phase |
| Score approach seconds | 2.00 | Seconds of drive before the shot |
| Score approach power | 0.30 | Signed motor duty for the approach |
| Score settle seconds | 0.25 | Seconds with all outputs stopped after the approach |
| Shot reverse seconds | 0.17 | Seconds with both rollers at -12 V |
| Shot spinup seconds | 0.60 | Seconds with the shooter powered and the intake stopped |
| Shot feed seconds | 1.00 | Seconds with the intake and shooter powered |
| Shot power | 1.00 | Fraction of 12 V for the spinup and feed phases |
| Feed voltage | 6.00 | Intake volts during the feed phase |
| Score exit seconds | 0.00 | Seconds of optional drive after the shot |
| Score exit power | 0.30 | Signed motor duty for the optional exit |

Drive duty is the requested motor fraction before controller inversion.
A value of 0.30 requests 30% duty on each side before trim.
Auto does not apply the 60% teleop multiplier to this value.
Positive drive values request forward motion. Negative values request reverse motion.

Trim uses `left = power + trim` and `right = power - trim`.
For example, power 0.30 and trim 0.02 request left 32% and right 28%.
During forward motion, positive trim turns the heading toward the right.
Trim is a fixed correction, not feedback from a sensor.

### Limits

Each duration must be finite and between 0 and 14.5 seconds.
The sum of the selected routine's durations must not exceed 14.5 seconds.
The remaining 0.5 seconds provide margin before the 15-second auto period ends.

Each drive power must be between -0.60 and 0.60.
Trim must be between -0.15 and 0.15.
During an active drive phase, neither side can exceed 0.60 after trim.
Trim cannot reverse one side against the requested direction.

Shot power must be between 0 and 1. Feed voltage must be between 0 and 12 V.
Invalid numbers, missing entries, and wrong entry types block the selected routine.
The robot stops all outputs instead of substituting another value.
Unused shot settings do not block Cross line. Do nothing always stops the outputs.

A zero duration skips that phase without a motor pulse.
A zero approach does not establish a legal shooting position.
The default start is outside the scoring zone, so the normal scoring routine needs a measured approach.

### Persistence and capture

The settings persist across robot restarts through NetworkTables.
The source defaults initialize missing entries. They do not overwrite retained settings.
At autonomous start, the robot captures the selected routine and its settings.
Edits during the run affect the next run only.

The main tab shows the routine and timing captured for the current run.
The Auto setup tab shows pending settings and their validation status.
The entry `/SmartDashboard/Auto/Next run seconds` gives the pending routine's total duration.

## Prepare the robot

1. Open `dashboard/elastic-layout.json` in Elastic.
2. Set the Driver Station Practice Mode periods to 15 seconds, 3 seconds, and 135 seconds.
3. Put the driver controller on USB 0 and the operator controller on USB 1.
4. Select Do nothing before the first enable.
5. Check the drive directions with the robot securely supported and its wheels clear.
6. Check that disabling the robot stops the drivetrain, intake, and shooter.
7. Check the intake and shooter directions separately in the team's normal test area.

Motor assignments in this source are left drive CAN 13, right drive CAN 15, intake CAN 3, and shooter CAN 2.
The right drive controller is inverted. The other three controllers are not inverted.

## Tune Cross line first

1. Mark a starting position inside your starting zone, near the truss and aligned with the goal.
2. Keep every bumper inside the starting zone.
3. Check clearance from the truss supports and the alliance-dividing midline.
4. Set Start delay seconds to 0 and Drive trim to 0.
5. Set Cross drive power to 0.30.
6. For the first floor run, reduce Cross drive seconds to 0.25.
7. Select Cross line.
8. Run autonomous from the same starting mark each time.
9. Record the distance, heading error, starting battery voltage, power, and duration.
10. Increase the duration in small increments, usually 0.10 to 0.20 seconds.
11. Once the travel is close, change only one setting per run.
12. If the robot drifts left, increase trim by 0.01 for the next run.
13. If the robot drifts right, decrease trim by 0.01 for the next run.
14. Stop increasing the duration after the robot enters the scoring zone with repeatable margin.
15. Repeat the final route with the intended preload.

Crossing needs only part of the robot inside the scoring zone.
A duration that earns crossing points can still leave the rear bumper outside the scoring zone.
The robot also needs enough clearance to stop without hitting field elements.

## Tune Cross and score

1. Use the same marked starting position and goal alignment.
2. Set Score exit seconds to 0.
3. Set Shot reverse seconds, Shot spinup seconds, and Shot feed seconds to 0 for the initial approach tests.
4. Set Score approach power to the tested crossing power.
5. Set Score approach seconds to the tested crossing duration.
6. Select Cross and score.
7. Run the approach without a shot.
8. Increase Score approach seconds until every bumper clears the scoring-zone boundary with margin.
9. Check that the stopped robot remains aligned with the goal.
10. Record the approach settings separately from Cross line.
11. Restore Shot reverse seconds to 0.17.
12. Restore Shot spinup seconds to 0.60.
13. Restore Shot feed seconds to 1.00.
14. Load one ball.
15. Run the full routine in the team's normal shooting area.
16. If the ball leaves too low or too high, adjust Shot power in small increments.
17. If the ball feeds before the shooter stabilizes, increase Shot spinup seconds.
18. If the ball stays in the intake, inspect the mechanism before increasing Feed voltage or Shot feed seconds.
19. Repeat the final routine from the same mark with representative battery voltages.

The code reports the spinup delay, not measured shooter speed.
The code also cannot detect a retained ball, a jam, a goal, or the robot's field position.
Optional exit motion needs a separate field check. It must remain in the team's own half during auto.

## Run the full Practice Mode sequence

1. Run the chosen routine through the full 15/3/135 Practice Mode sequence.
2. Check that the robot stops after the routine and remains stopped for the rest of auto.
3. Check that joystick movement during auto does not change its path.
4. Check that teleop begins with the usual arcade controls and the 60% drive limit.
5. Disable once during a drive phase.
6. Disable once during a shot phase in the normal shooting area.
7. Check that each disable stops all outputs.
8. Check the dashboard's selected XYAB letter and percentage.
9. During a shot, change XYAB.
10. Check that only the next shot uses the new selection.
11. Before each match, check the selected auto and the pending settings.

If scoring is inconsistent, select the tested Cross line routine.
If crossing is also inconsistent, select Do nothing.

## Stops and error messages

| Status | Meaning and action |
| --- | --- |
| Invalid settings / Blocked | An entry fails validation. Correct the named entry while disabled. |
| Loop overrun | More than 100 ms passed between auto updates. The routine stops instead of skipping into a later phase. Inspect robot logs. |
| Clock error | The elapsed clock moved backward or became invalid. The routine stops all outputs. |
| Interrupted | A cancellation, disable, or conflicting command stopped the routine. All outputs are stopped. |
| Complete | The timed sequence ended. This status does not establish position or a scored ball. |

The 100 ms stop prevents a delayed loop from jumping past shooter spinup into feeding.
The command also refreshes motor requests each loop and stops them after every end or interruption.

## Tests

Run `./gradlew test` with the project's WPILib 2026 toolchain.
The automated tests cover timing, sequence order, interruptions, invalid settings, chooser values, captured settings, and output cleanup.
They also cover zero-duration phases, repeated runs, subsystem ownership, and delayed scheduler updates.
Physical travel, preload retention, truss clearance, and scoring accuracy still need field tests.
