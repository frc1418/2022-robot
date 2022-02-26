package frc.robot.commands.autonomous;

import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.DriveSubsystem;

public class AroundTarmacCommand extends SequentialCommandGroup{
    private String TRAJECTORY_NAME = "aroundTarmac";

    public AroundTarmacCommand(DriveSubsystem driveSubsystem, Odometry odometry, HashMap<String, Trajectory> trajectories) {
        Trajectory aroundTarmac = trajectories.get(TRAJECTORY_NAME);

        addCommands(
            new PrintCommand("BEGINING AROUND TARMAC"),
            new FollowTrajectoryCommand(aroundTarmac, odometry, driveSubsystem, true),
            new PrintCommand("ENDING AROUND TARMAC"));
    }
}
