package frc.robot.common;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class TrajectoryLoader {

    private static final String OUTPUT_DIRECTORY_NAME = "paths";

    public TrajectoryLoader() {}

    private List<File> getTrajectoryFiles() {
        File pathsDirectory =
                Filesystem.getDeployDirectory().toPath().resolve(OUTPUT_DIRECTORY_NAME).toFile();
        File[] trajectoryFiles = pathsDirectory.listFiles();

        return Arrays.asList(trajectoryFiles);
    }

    public HashMap<String, Trajectory> loadTrajectories() {
        HashMap<String, Trajectory> trajectories = new HashMap<>();
        List<File> files = getTrajectoryFiles();
        for (File file : files) {
            Trajectory trajectory = new Trajectory();
            try {
                trajectory = TrajectoryUtil.fromPathweaverJson(file.toPath());
                trajectories.put(file.getName().split("\\.")[0], trajectory);
            } catch (IOException ex) {
            }
        }
        return trajectories;
    }
}