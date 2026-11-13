
 // Copyright (c) FIRST and other WPILib contributors.
 // Open Source Software; you can modify and/or share it under
 // the terms of the WPILib BSD license file in the root directory.

package frc.robot.drive;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class KitbotTankDrivetrain extends SubsystemBase {

    // Drive motors
    private final WPI_TalonSRX leftDrive = new WPI_TalonSRX(13);
    private final WPI_TalonSRX rightDrive = new WPI_TalonSRX(15);

    // WPILib differential drive
    private final DifferentialDrive drive;

    public KitbotTankDrivetrain() {

        // Reset motor controllers to factory defaults
        leftDrive.configFactoryDefault();
        rightDrive.configFactoryDefault();

        // Set motor directions
        leftDrive.setInverted(false);
        rightDrive.setInverted(true);

        // Set brake mode
        leftDrive.setNeutralMode(NeutralMode.Brake);
        rightDrive.setNeutralMode(NeutralMode.Brake);

        // Initialize differential drive
        drive = new DifferentialDrive(leftDrive, rightDrive);

        // Deadband handled manually
        drive.setDeadband(0.0);

        // Set maximum drive output
        drive.setMaxOutput(Constants.DRIVE_MAX_OUTPUT);
    }

    /**
     * Arcade drive command.
     *
     * @param speed Forward/backward joystick input
     * @param rotation Turning joystick input
     */
    public Command driveArcade(
            DoubleSupplier speed,
            DoubleSupplier rotation) {

        return this.runEnd(
            () -> {

                // Apply forward/backward deadband
                double forward = MathUtil.applyDeadband(
                    speed.getAsDouble(),
                    Constants.DRIVE_DEADBAND
                );

                // Apply turning deadband
                double turn = MathUtil.applyDeadband(
                    rotation.getAsDouble(),
                    Constants.AIM_DEADBAND
                );

                // Drive using WPILib arcade drive
                drive.arcadeDrive(forward, turn, false);
            },

            // Stop drivetrain when command ends
            this::brake
        );
    }

    /**
     * Compatibility method.
     * Existing calls to driveTank will now use arcade drive.
     *
     * @param leftSpeed Forward/backward joystick input
     * @param rightSpeed Unused; retained for compatibility
     * @param aim Turning joystick input
     */
    public Command driveTank(
            DoubleSupplier leftSpeed,
            DoubleSupplier rightSpeed,
            DoubleSupplier aim) {

        return driveArcade(leftSpeed, aim);
    }

    /**
     * Stop drivetrain motors.
     */
    public void brake() {
        drive.stopMotor();
    }

    @Override
    public void periodic() {
        // No periodic functionality currently needed
    }
}
