package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class TrajectorySubsystem extends SubsystemBase {

        // left
        private static Path LeftPickUpPath = Filesystem.getDeployDirectory().toPath()
                        .resolve("paths/Left Pick Up.wpilib.json");
        
        private static Path LeftScorePath = Filesystem.getDeployDirectory().toPath()
                        .resolve("paths/Left Score.wpilib.json");
        

        // center
        private static Path CenterPickUpPath = Filesystem.getDeployDirectory().toPath()
                        .resolve("paths/Center Pick Up.wpilib.json");
        
        private static Path CenterScorePath = Filesystem.getDeployDirectory().toPath()
                        .resolve("paths/Center Score.wpilib.json");
        
        private static Path CenterPickUpMorePath = Filesystem.getDeployDirectory().toPath()
                        .resolve("paths/Center Pick Up More.wpilib.json");
        
        private static Path CenterScoreMorePath = Filesystem.getDeployDirectory().toPath()
                        .resolve("paths/Center Score More.wpilib.json");
        

        // right
        private static Path RightPickUpPath = Filesystem.getDeployDirectory().toPath()
                        .resolve("paths/Right Pick Up.wpilib.json");
        
        private static Path RightScorePath = Filesystem.getDeployDirectory().toPath()
                        .resolve("paths/Right Score.wpilib.json");


        private static TrajectoryConfig config = // create the config used for all trajectories
            new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                    DriveConstants.kMaxAccelerationMetersPerSecondSquared)
                            // Add kinematics to ensure max speed is actually obeyed
                            .setKinematics(DriveConstants.kDriveKinematics);
                            
        //Blue Left
        public static Trajectory LeftPickUp;
        public static Trajectory LeftScore;

        // Blue Center
        public static Trajectory CenterPickUp;
        public static Trajectory CenterScore;
        public static Trajectory CenterPickUpMore;
        public static Trajectory CenterScoreMore;

        // Blue Right
        public static Trajectory RightPickUp;
        public static Trajectory RightScore;
        

        public TrajectorySubsystem() {

                //left
                try {
                        LeftPickUp = TrajectoryUtil.fromPathweaverJson(LeftPickUpPath);
                } catch (IOException ex) {
                        DriverStation.reportError("Unable to open trajectory: " + LeftPickUpPath.toString(),
                                        ex.getStackTrace());
                }
                try {
                        LeftScore = TrajectoryUtil.fromPathweaverJson(LeftScorePath);
                } catch (IOException ex) {
                        DriverStation.reportError("Unable to open trajectory: " + LeftScorePath.toString(),
                                        ex.getStackTrace());
                }

                //center
                try {
                        CenterPickUp = TrajectoryUtil.fromPathweaverJson(CenterPickUpPath);
                } catch (IOException ex) {
                        DriverStation.reportError("Unable to open trajectory: " + CenterPickUpPath.toString(),
                                        ex.getStackTrace());
                }
                try {
                        CenterScore = TrajectoryUtil.fromPathweaverJson(CenterScorePath);
                } catch (IOException ex) {
                        DriverStation.reportError("Unable to open trajectory: " + CenterScorePath.toString(),
                                        ex.getStackTrace());
                }
                try {
                        CenterPickUpMore = TrajectoryUtil.fromPathweaverJson(CenterPickUpMorePath);
                } catch (IOException ex) {
                        DriverStation.reportError("Unable to open trajectory: " + CenterPickUpMorePath.toString(),
                                        ex.getStackTrace());
                }
                try {
                        CenterScoreMore = TrajectoryUtil.fromPathweaverJson(CenterScoreMorePath);
                } catch (IOException ex) {
                        DriverStation.reportError("Unable to open trajectory: " + CenterScoreMorePath.toString(),
                                        ex.getStackTrace());
                }

                //right
                try {
                        RightPickUp = TrajectoryUtil.fromPathweaverJson(RightPickUpPath);
                } catch (IOException ex) {
                        DriverStation.reportError("Unable to open trajectory: " + RightPickUpPath.toString(),
                                        ex.getStackTrace());
                }
                try {
                        RightScore = TrajectoryUtil.fromPathweaverJson(RightScorePath);
                } catch (IOException ex) {
                        DriverStation.reportError("Unable to open trajectory: " + RightScorePath.toString(),
                                        ex.getStackTrace());
                }
        }
        
        public Trajectory getLeftPickUp() {
                return LeftPickUp;
        }

        public Trajectory getLeftScore() {
                return LeftScore;
        }

        public Trajectory getCenterPickUp() {
                return CenterPickUp;
        }

        public Trajectory getCenterScore() {
                return CenterScore;
        }

        public Trajectory getCenterPickUpMore() {
                return CenterPickUpMore;
        }

        public Trajectory getCenterScoreMore() {
                return CenterScoreMore;
        }

        public Trajectory getRightPickUp() {
                return RightPickUp;
        }

        public Trajectory getRightScore() {
                return RightScore;
        }

        public static Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 0)),
            // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(2, 0, new Rotation2d(0)), config);

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
