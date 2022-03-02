package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
    public static final class DriveConstants{

        //wheel ports
        public static final int kFrontLeftWheelPort = 2;
        public static final int kFrontRightWheelPort = 3;
        public static final int kBackLeftWheelPort = 6;
        public static final int kBackRightWheelPort = 4;

        //drivetrain stuff
        public static double speed = 0.5; //speed control 

        public static double ks = 0.10049;
        public static double kv = 2.5175;
        public static double ka = 0.46019;
        public static final double kTrackWidth = 0.585; //23 in
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.52; //20.5 in
        // Distance between centers of front and back wheels on robot
        
        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 0.0017676;

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 0.0017676;
        public static final double kPYController = 0.0017676;
        public static final double kPThetaController = 0.0017676;

        public static final MecanumDriveKinematics kDriveKinematics =
        new MecanumDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final SimpleMotorFeedforward kFeedforward =
        new SimpleMotorFeedforward(ks, kv, ka);

        public static final PIDController driveController = new PIDController(kPDriveVel, 0, 0);
        
         // Constraint for the motion profilied robot angle controller
         public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
         new TrapezoidProfile.Constraints(
             kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final PIDController xController = new PIDController(kPXController, 0, 0);

        public static final PIDController yController = new PIDController(kPYController, 0, 0);

        public static final ProfiledPIDController thetaController
         = new ProfiledPIDController(kPThetaController, 0, 0, kThetaControllerConstraints);
    
    }
    public static final class IntakeConstants {
        public static final int kIntakePort = 5;
        public static final double kIntakeSpeed = 0.9;

        public static final int kIntakeDropPort = 13;
        public static final double kIntakeDropSpeed = 0.6;
        public static final int kIntakeDropTime = 1000;
    }

    public static final class ConveyorConstants{
        public static final int kConveyorPort = 1;
        public static final double kConveyorSpeed = 0.8;
    }

    public static final class FeederConstants{
        public static final int kFeederPort = 11;
        public static final double kFeederSpeed = 0.4;
    }

    public static final class LiftConstants{
        public static final int kLift1Port = 8;
        public static final int kLift2Port = 7;

        public static final double kIdealLiftSpeed = -0.25;

        public static final int bottomPose = 0; //bottom limit
        public static final int topPose = 137; //top limit

        public static final double liftSpeed = 100;

        public static final double kP = 0.04;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kMaxLiftPower = 0.6;
    }

    public static final class TurretConstants{
        public static final int kTurretPort = 12;

        public static final double kTurretSpeed = 0.3;
    }

    public static final class ShooterConstants {
        public static final int kShooter1Port = 10;
        public static final int kShooter2Port = 9;

        public static final int kServoPort = 1;

        public static final double kIdealShotSpeed = 0.5; //ideal motor speed to run the shooter at
        public static final double kTimeToChargeUp = 4; //time in seconds for the shooter to reach ideal speed

        public static final double shooterSpeed = 6000;

        public static final double kS = 0; //0.052824
        public static final double kV = 0; //0.12603
        public static final double kA = 0; //0.0055052

        public static final double kP = 0.002; //2.5946 * Math.pow(10, -10)
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 8.6;

        public static TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(10000, 1000);
    }
    public static final class JoystickConstants {
        public static final int kXStick1 = 0;
        public static final int kYStick1 = 1;
        public static final int kLeftTrigger = 2;
        public static final int kRightTrigger = 3;
        public static final int kXStick2 = 4;
        public static final int kYStick2 = 5;

        public static final int kJoystick1Port = 0;
        public static final int kJoystick2Port = 2;
    }

    public static final class LimelightConstants {
        public static final double kIdealStrafeValue = 0.4;
        public static final double kIdealForwardValue = 0.2;
        public static final double kIdealRotateValue = 0.15;

        public static final double kIdealAreaValue = 2.5;
        public static final double kAreaRangeValue = 0.3;

        public static final double kP = 0.005;

        public final static double STEER_K = 0.02; // how hard to turn toward the target
        public final static double DRIVE_K = 0.1; // how hard to drive fwd toward the target
        public final static double DESIRED_TARGET_AREA = 3.0; // Area of the target when the robot reaches the wall
        public final static double MAX_DRIVE = 0.6; // Simple speed limit so we don't drive too fast
    }

    public static final class SonarConstants{
        public static final int sonar1 = 0;
        public static final int sonar2 = 1;
    }
    public static final class AutoConstants {
        public static final double kAutoShootPower = -0.9;
        public static final double kAutoChargeUpTime = 2;
        public static final double kAutoShootEndTime = 5;

      }

    
}
