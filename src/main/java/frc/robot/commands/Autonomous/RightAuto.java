package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoShooterCommand;
import frc.robot.commands.ConveyorCommand;
import frc.robot.commands.DropIntakeDelay;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SetDrivePose;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TrajectorySubsystem;

public class RightAuto extends SequentialCommandGroup {

        TrajectorySubsystem m_trajectories;

    public RightAuto(DriveSubsystem drive, IntakeSubsystem intake, ConveyorSubsystem conveyor, ShooterSubsystem shooter,
                    FeederSubsystem feeder, LimelightSubsystem camera, BlinkinSubsystem led,
                    TrajectorySubsystem trajectories) {
                            m_trajectories = trajectories;
                            addCommands(
                                            new SetDrivePose(drive, trajectories.getRightPickUp()),
                                            new DropIntakeDelay(intake, IntakeConstants.kIntakeDropSpeed, IntakeConstants.kIntakeDropTime),
                            new IntakeCommand(intake, IntakeConstants.kIntakeSpeed),
                            new ConveyorCommand(conveyor, ConveyorConstants.kConveyorSpeed),
                            new ShooterCommand(shooter, camera, ShooterConstants.shooterSpeed
                                                            * camera.getShooterSpeed()),
                            new MecanumControllerCommand(
                                            trajectories.getRightPickUp(),
                                            drive::getPose,
                                            DriveConstants.kFeedforward,
                                            DriveConstants.kDriveKinematics,
                                            // Position contollers
                                            DriveConstants.xController,
                                            DriveConstants.yController,
                                            DriveConstants.thetaController,
                                            // Needed for normalizing wheel speeds
                                            DriveConstants.kMaxSpeedMetersPerSecond,
                                            // Velocity PID's
                                            DriveConstants.driveController,
                                            DriveConstants.driveController,
                                            DriveConstants.driveController,
                                            DriveConstants.driveController,
                                            drive::getCurrentWheelSpeeds,
                                            drive::setDriveMotorControllersVolts, // Consumer for the output motor voltages
                                            drive),
                        //     new MecanumControllerCommand(
                        //                     trajectories.getRightScore(),
                        //                     drive::getPose,
                        //                     DriveConstants.kFeedforward,
                        //                     DriveConstants.kDriveKinematics,
                        //                     // Position contollers
                        //                     DriveConstants.xController,
                        //                     DriveConstants.yController,
                        //                     DriveConstants.thetaController,
                        //                     // Needed for normalizing wheel speeds
                        //                     DriveConstants.kMaxSpeedMetersPerSecond,
                        //                     // Velocity PID's
                        //                     DriveConstants.driveController,
                        //                     DriveConstants.driveController,
                        //                     DriveConstants.driveController,
                        //                     DriveConstants.driveController,
                        //                     drive::getCurrentWheelSpeeds,
                        //                     drive::setDriveMotorControllersVolts, // Consumer for the output motor voltages
                                            //                     drive),
                                new ShooterCommand(shooter, camera, ShooterConstants.shooterSpeed
                                                            * camera.getShooterSpeed()),
                            
                                            new AutoShooterCommand(shooter, conveyor, feeder, led, camera),
                                            new IntakeCommand(intake, 0),
                            new ConveyorCommand(conveyor, 0),
                            new ShooterCommand(shooter, camera, 0),
                            new FeederCommand(feeder, 0));
    }
    
    public Pose2d getPose() {
            return m_trajectories.getRightPickUp().getInitialPose();
    }
}
