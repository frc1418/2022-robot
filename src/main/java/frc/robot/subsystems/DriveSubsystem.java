// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.DriverAction;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.Odometry;

public class DriveSubsystem extends SubsystemBase {
  
    private final DifferentialDrive driveTrain;
    private final MotorControllerGroup leftMotors;
    private final MotorControllerGroup rightMotors;
    private final Odometry odometry;
    private final Field2d field;
    private final Timer timer;

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

    private final NetworkTable table = ntInstance.getTable("/common/Odometry");
    private final NetworkTableEntry odometryPose = table.getEntry("odometryPose");
    private final NetworkTableEntry encoderPosition = table.getEntry("encoderPose");
    private final NetworkTableEntry leftEncoderPosition = table.getEntry("leftEncoderPos");
    private final NetworkTableEntry rightEncoderPosition = table.getEntry("rightEncoderPos");

    private final NetworkTableEntry time = table.getEntry("time");

    public DriveSubsystem(DifferentialDrive driveTrain, MotorControllerGroup leftMotors, MotorControllerGroup rightMotors, Odometry odometry, Field2d field, Timer timer) {
        this.driveTrain = driveTrain;
        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;
        this.odometry = odometry;
        this.field = field;
        this.timer = timer;

        SmartDashboard.putData("Field", field);
    }

    public void joystickDrive(double speed, double rotation) {
        driveTrain.setDeadband(RobotDriveBase.kDefaultDeadband);
        driveTrain.arcadeDrive(-speed, rotation);
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

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(rightVolts);
        driveTrain.feed();
    }
   
    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometryPose.setString(odometry.getPose().toString());
        field.setRobotPose(odometry.getPose());
        odometry.update();
        encoderPosition.setDouble(odometry.getAverageEncoderDistance());
        time.setDouble(timer.get());
        leftEncoderPosition.setDouble(odometry.getLeftEncoderDistance());
        rightEncoderPosition.setDouble(odometry.getRightEncoderDistance());
    }
}
