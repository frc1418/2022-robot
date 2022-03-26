package frc.robot.commands.autonomous;

import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveTrain;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.DriveSubsystem;

public class ChargeCommand extends SequentialCommandGroup{
    private String TRAJECTORY_NAME = "straightBack";

    public ChargeCommand(DriveSubsystem driveSubsystem, Odometry odometry, HashMap<String, Trajectory> trajectories) {
        Trajectory charge = trajectories.get(TRAJECTORY_NAME);

        addCommands(
            new FollowTrajectoryCommand(charge, odometry, driveSubsystem, true));
    }
}
