package frc.robot.commands.autonomous;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrain;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.DriveSubsystem;

public class SlalomCommand extends SequentialCommandGroup{
    private String TRAJECTORY_NAME = "curve";

    public SlalomCommand(DriveSubsystem driveSubsystem, Odometry odometry, HashMap<String, Trajectory> trajectories) {
       
    
        Trajectory slalom = trajectories.get(TRAJECTORY_NAME);


        addCommands(
            new FollowTrajectoryCommand(slalom, odometry, driveSubsystem, true));
    }
}
