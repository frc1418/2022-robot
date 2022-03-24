// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


    public static final double metersToInches = 39.37;

    /**
     * 
     * CAN IDs
     * 
     * Drive Train:
         * Front Right: 1
         * Rear Right: 2
         * Front Left: 3
         * Rear Left: 4
     * Shooter: 5
     * Intake: 6
     * Storage:
         * Right: 7
         * Left: 8
     * Climber: 9
     * 
     * 
     * Solenoid Ports
     * 
     * Shooter:
         * Forward: 6
         * Reverse: 7
     * Intake:
         * Right:
             * Forward: 0
             * Reverse: 1
         * Left:
             * Forward: 2
             * Reverse: 3  
     */
    public static final class DriveTrain {
        public static final int FRONT_LEFT_MOTOR = 25;
        public static final int FRONT_RIGHT_MOTOR = 3;
        public static final int REAR_LEFT_MOTOR = 2;
        public static final int REAR_RIGHT_MOTOR = 4;
        
        public static final double DRIVE_GEARING = 10.71;
        public static final double DRIVE_WHEEL_DIAMETER = Units.inchesToMeters(6);  // Meters
        public static final double DRIVE_ENCODER_CONSTANT = (1 / DRIVE_GEARING) * DRIVE_WHEEL_DIAMETER * Math.PI;

        public static final double MAX_GENERATION_VOLTAGE = 10;  // Volts
        public static final double MAX_GENERATION_VELOCITY = 3.25;  // Meters per second
        public static final double MAX_GENERATION_ACCELERATION = 2.2;  // Meters per second squared


        // DriveSubsystem constants
        private static final double METERS_PER_ROTATION = Math.PI * DRIVE_WHEEL_DIAMETER;  // Meters
        public static final double TRACK_WIDTH = 0.72248;  // Meters (This value was given by SysID)
        public static final double WHEEL_BASE = 0.4469;  // Meters

        public static final double KS = 0.18838;  // Volts
        public static final double KV = 2.7771;  // Volt seconds per meter
        public static final double KA = 0.42762;  // Volt seconds squared per meter
        public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);
        public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(KS, KV, KA);
    }

    public static final class Shooter {
        public static final int SHOOTER_MOTOR = 5;
        public static final int SHOOTER_SOLENOID_FWD = 6;
        public static final int SHOOTER_SOLENOID_REV = 7;

        public static final double kP = 0.00023;
        public static final double kI = 0.00000001;
        public static final double kD = 0.00000001;
        public static final double kF = 0.000184;

        public static final int SHOOTER_RATIO = 4;
    }

    public static final class Intake {
        public static final int INTAKE_MOTOR = 6;
        public static final int SOLENOID_RIGHT_FWD = 1;
        public static final int SOLENOID_RIGHT_REV = 0;
        public static final int SOLENOID_LEFT_FWD = 2;
        public static final int SOLENOID_LEFT_REV = 3;
    }

    public static final class Storage {
        public static final int STORAGE_RIGHT_MOTOR = 7;
        public static final int STORAGE_LEFT_MOTOR = 8;
    }

    public static final class Climber {
        public static final int CLIMBER_MOTOR = 9;
        public static final int MEDIUM_RUNG_POS = 102;
        public static final int CLIMBER_DOWN_POS = 1;

        public static final int CLICKS_PER_ROTAITON = 2048;
    }
  
    public static final class Limelight {
        // UNIT = inches
        public static final double CAMERA_ELEVATION = 30.75; // TEST BOT
        public static final double TARGET_ELEVATION = 258; // TEST BOT
        public static final double LIMELIGHT_TO_ROBOT_CENTER = 9; // TEST BOT
        // UNIT = degrees
        public static final double CAMERA_ANGLE = 0; // TEST BOT


        // LED MODES /limelight/ledMode
        public static final int LED_MODE_FROM_PIPELINE = 0;
        public static final int LED_MODE_FORCE_OFF = 1;
        public static final int LED_MODE_FORCE_BLINK = 2;
        public static final int LED_MODE_FORCE_ON = 3;

        // pipeline numbers
        public static final int PIPELINE_TELEOP = 0;
        public static final int PIPELINE_GET_POS = 1;
    }

    public static final class Align {
        public static final double positionTolerance = 3.5;
        public static final double velocityTolerance = 0.1;
    
        public static final double integratorMin = -4.0;
        public static final double integratorMax = 4.0;
    }

    public static final class DriverValues {
        public static final int intakeInVoltage = -9;
        public static final int intakeOutVoltage = 9;

        public static final int shooterHighVelocity = -2000;
        public static final int shooterLowVelocity = -1200;
        public static final int shooterBackVoltage = 3;

        public static final double storageInVoltage = 2.5;
        public static final double storageOutVoltage = -2.5;

        public static final double climberUpVoltage = 0.5;
        public static final double climberDownVoltage = -0.7;
    }

    public static final class AutomonousValues {

        public static final int autoShooterVelocity = -1200;
        public static final double autoStorageVolts= 2.5;
        public static final double autoShootTime = 3;
        public static final double autoShootTimeout = 6;
    }
  
    public static final int EXTRA_CAN_ID = 0;
    public static final int EXTRA_CAN_ID_2 = 24;
}
