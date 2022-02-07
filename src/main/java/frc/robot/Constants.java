// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

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
        public static final int FRONT_LEFT_MOTOR = 3;
        public static final int FRONT_RIGHT_MOTOR = 1;
        public static final int REAR_LEFT_MOTOR = 4;
        public static final int REAR_RIGHT_MOTOR = 2;
    }

    public static final class Shooter {
        public static final int SHOOTER_MOTOR = 5;
        public static final int SHOOTER_SOLENOID_FWD = 6;
        public static final int SHOOTER_SOLENOID_REV = 7;
        public static final int TARMAC_LINE_VEL = 2100;
    }

    public static final class Intake {
        public static final int INTAKE_MOTOR = 6;
        public static final int SOLENOID_RIGHT_FWD = 0;
        public static final int SOLENOID_RIGHT_REV = 1;
        public static final int SOLENOID_LEFT_FWD = 2;
        public static final int SOLENOID_LEFT_REV = 3;
    }

    public static final class Storage {
        public static final int STORAGE_RIGHT_MOTOR = 7;
        public static final int STORAGE_LEFT_MOTOR = 8;
    }

    public static final class Climber {
        public static final int CLIMBER_MOTOR = 9;
    }

    public static final int EXTRA_CAN_ID = 25;
}
