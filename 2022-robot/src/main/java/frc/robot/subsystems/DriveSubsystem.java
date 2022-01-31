// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  
    private final DifferentialDrive driveTrain;

    public DriveSubsystem(DifferentialDrive driveTrain) {
        this.driveTrain = driveTrain;
    }

    public void joystickDrive(double speed, double rotation) {
        driveTrain.setDeadband(RobotDriveBase.kDefaultDeadband);
        driveTrain.arcadeDrive(speed, rotation);
    }

    /**
     * The {@link #joystickDrive(double, double) joystickDrive} method with a
     * fixed deadband of 0 and without squared inputs.
     * 
     * @param speed
     * @param rotation
     */
    public void drive(double speed, double rotation) {
        driveTrain.setDeadband(0);
        driveTrain.arcadeDrive(speed, rotation, false);
    }
}
