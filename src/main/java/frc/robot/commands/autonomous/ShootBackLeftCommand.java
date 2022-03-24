package frc.robot.commands.autonomous;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.ShootyCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.Constants.AutomonousValues;
import frc.robot.Constants.DriveTrain;

public class ShootBackLeftCommand extends SequentialCommandGroup {

    private String TRAJECTORY_NAME = "backLeft";


    public ShootBackLeftCommand(DriveSubsystem driveSubsystem, Odometry odometry,
            StorageSubsystem storageSubsystem, ShooterSubsystem shooterSubsystem,
            HashMap<String, Trajectory> trajectories){


        TrajectoryConfig forwardConfig =
            new TrajectoryConfig(DriveTrain.MAX_GENERATION_VELOCITY, DriveTrain.MAX_GENERATION_ACCELERATION)
                .setKinematics(DriveTrain.KINEMATICS)
                .addConstraint(
                    new DifferentialDriveVoltageConstraint(
                        DriveTrain.FEED_FORWARD,
                        DriveTrain.KINEMATICS,
                        DriveTrain.MAX_GENERATION_VOLTAGE));
        forwardConfig.setReversed(true);
        
        // Trajectory backLeft =
        //     TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(4.911015989056086, -4.515522352941174, new Rotation2d(0)),
        //         List.of(
        //                 new Translation2d(4.711015989056086, -4.515522352941174)),
        //         new Pose2d(7.259673028727769, -3.748139359781121, new Rotation2d(0)),
        //         forwardConfig);
        
        Trajectory backLeft = trajectories.get(TRAJECTORY_NAME);


        addCommands(
            new ShootyCommand(AutomonousValues.autoShooterVelocity,
                    AutomonousValues.autoStorageVolts, AutomonousValues.autoShootTime,
                    AutomonousValues.autoShootTimeout, shooterSubsystem, storageSubsystem),
            new FollowTrajectoryCommand(backLeft, odometry, driveSubsystem, true));

    }
    
}
