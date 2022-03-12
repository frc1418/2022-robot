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

import frc.robot.Constants.DriveTrain;;

public class ShootBackRightCommand extends SequentialCommandGroup {

    private String TRAJECTORY_NAME = "backRight";


    public ShootBackRightCommand(DriveSubsystem driveSubsystem, Odometry odometry,
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
        
        // Trajectory backRight =
        //     TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(6.003955403556771, -2.445913674418604, new Rotation2d(0)),
        //         List.of(
        //                 new Translation2d(5.803955403556771, -2.445913674418604)),
        //         new Pose2d(7.213164968536251, -3.655123239398084, new Rotation2d(0)),
        //         forwardConfig);
        
        Trajectory backRight = trajectories.get(TRAJECTORY_NAME);


        addCommands(
            new ShootyCommand(-2000, 2.5, 3, 6, shooterSubsystem, storageSubsystem),
            new FollowTrajectoryCommand(backRight, odometry, driveSubsystem, true));

    }
    
}
