package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import io.github.oblarg.oblog.annotations.Config;

public class Constants {
    public static final class DriveConstants{

        //wheel ports
        public static final int kFrontLeftWheelPort = 2;
        public static final int kFrontRightWheelPort = 3;
        public static final int kBackLeftWheelPort = 6;
        public static final int kBackRightWheelPort = 4;


        public static final int kLeftMotorPort = 2; 
        public static final int kRightMotorPort = 6; 
        
        public static final double metersPerRotation = 0.4787787204060999;

        public static final double gearRatio = 10.71;

        public static final double encoderConversionFactor = metersPerRotation / gearRatio;

        public static final double encoderVeloConversionFactor = metersPerRotation / gearRatio / 60;
        //drivetrain stuff
        public static double speed = 0.5; //speed control 

        public static double ks = 0.079775;
        public static double kv = 2.9146;
        public static double ka = 0.65184;
        public static final double kTrackWidth = 0.585; //23 in
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.52; //20.5 in
        // Distance between centers of front and back wheels on robot
        
        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 0.00028512;

        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 0.00028512;
        public static final double kPYController = 0.00028512;
        public static final double kPThetaController = 0.00028512;

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
        public static final double kIntakeSpeed = 1;

        public static final int kIntakeDropPort = 13;
        public static final double kIntakeDropSpeed = 0.6;
        public static final double kIntakeHoldSpeed = 0.1;
        public static final int kIntakeDropTime = 1000;
    }

    public static final class ConveyorConstants{
        public static final int kConveyorPort = 1;
        public static final double kConveyorSpeed = 0.7;
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
        public static final int topPose = 150; //top limit

        public static final double liftSpeed = 100;

        public static final double kP = 0.06;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kMaxLiftPower = 1;
    }

    public static final class TurretConstants{
        public static final int kTurretPort = 12;

        public static final double kTurretLimit = 9;

        public static final double kTurretSpeed = 0.5;
    }

    public static final class ShooterConstants {
        public static final int kShooter1Port = 10;
        public static final int kShooter2Port = 9;

        public static final int kServoPort = 1;

        public static final double kIdealShotSpeed = 0.5; //ideal motor speed to run the shooter at
        public static final double kTimeToChargeUp = 4; //time in seconds for the shooter to reach ideal speed

        @Config
        public static final double shooterSpeed = 2000; 

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
        public static final int kJoystick2Port = 1;
    }

    public static final class LimelightConstants {
        public static final double kIdealStrafeValue = 0.4;
        public static final double kIdealForwardValue = 0.2;
        public static final double kIdealRotateValue = 0.15;

        public static final double kIdealAreaValue = 2.5;
        public static final double kAreaRangeValue = 0.3;

        public static final double kP = 0.005;

        public final static double STEER_K = 0.05; // how hard to turn turret
        public final static double SHOOTER_K = 1.6; // change power of shooter to reach target //1.75
        public final static double SHOOTER_F = 0;
        public final static double DESIRED_TARGET_AREA = 0.75; // Area of the target when the robot reaches the wall
        public final static double MAX_DRIVE = 0.6; // Simple speed limit so we don't drive too fast
    }

    public static final class SonarConstants{
        public static final int sonar1 = 0;
        public static final int sonar2 = 1;
    }

    public static final class BlinkinConstants{
        public static final int kBlinkinPort = 0;

        //colors
        public static final double kRed = 0.61;
        public static final double kOrange = 0.65;
        public static final double kYellow = 0.69;
        public static final double kGreen = 0.77;
        public static final double kBlue = 0.87;
        public static final double kViolet = 0.91;
        public static final double kWhite = 0.93;
        public static final double kBlack = 0.99;
    }
    public static final class AutoConstants {
        public static final double kAutoShootPower = -0.9;
        public static final double kAutoChargeUpTime = 2;
        public static final double kAutoShootEndTime = 5;

      }

    
}
