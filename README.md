# Fall 2026 Mock Season

Titan Robotics starter for the 2026 Gear Up Games, preparing for the 2027 FRC season.
This local repository uses the **same template architecture as FRC-2026-JAVA**.

Hardware is unconfigured.

## Build and inspect

Use WPILib 2026 / Java 17. The project's GradleRIO version is 2026.2.1.

```bash
export JAVA_HOME="$HOME/wpilib/2026/jdk"  # this Linux workstation
./gradlew build
./gradlew test
python3 scripts/verify_originals.py
./gradlew simulateJava -Pheadless
```

Windows: use the WPILib terminal and `gradlew.bat`. Python is only needed for the
optional byte-identity audit, not for building the robot. The first build may
download the copied vendor dependencies. Headless simulation starts a stopped,
unconfigured robot; it does not simulate a driving or ball-handling mechanism.
Ctrl+C ends it. Interactive simulation uses `./gradlew simulateJava` on your own
or an isolated desktop.