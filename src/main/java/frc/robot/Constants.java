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
         * Front: 7
         * Back: 8
     * Climber: 9
     */
    public static final class DriveTrain {
        public static final int FRONT_LEFT_MOTOR = 3;
        public static final int FRONT_RIGHT_MOTOR = 1;
        public static final int REAR_LEFT_MOTOR = 4;
        public static final int REAR_RIGHT_MOTOR = 2;
    }

    public static final class Shooter {
        public static final int SHOOTER_MOTOR = 5;
        public static final int SHOOTER_SOLENOID_PORT = 0;
    }

    public static final class Intake {
        public static final int INTAKE_MOTOR = 6;
    }

    public static final class Storage {
        public static final int STORAGE_FRONT_MOTOR = 7;
        public static final int STORAGE_BACK_MOTOR = 8;
    }

    public static final class Climber {
        public static final int CLIMBER_MOTOR = 9;
    }

}
