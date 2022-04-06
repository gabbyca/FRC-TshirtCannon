package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.DriveConstants;

public class Trajectories {

    private static TrajectoryConfig config = // create the config used for all trajectories
            new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                    DriveConstants.kMaxAccelerationMetersPerSecondSquared)
                            // Add kinematics to ensure max speed is actually obeyed
                            .setKinematics(DriveConstants.kDriveKinematics);

    public static Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(2, 1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(15, 2, new Rotation2d(0)), config);

    public static Trajectory anotherTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 2)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(2, 5, new Rotation2d(0)),
            config);
}
