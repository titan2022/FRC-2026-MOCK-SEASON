# Fall 2026 Mock Season

Titan Robotics starter for the 2026 Gear Up Games, preparing for the 2027 FRC season.
This local repository uses the same template architecture as FRC-2026-JAVA.

Hardware is somewhat tbd.

## Build and inspect

Use WPILib 2026 / Java 17. The project's GradleRIO version is 2026.2.1.

```bash
export JAVA_HOME="$HOME/wpilib/2026/jdk"  # this Linux workstation
./gradlew build
./gradlew simulateJava -Pheadless
```

Windows: use the WPILib terminal and `gradlew.bat`. The first build may
download the copied vendor dependencies.