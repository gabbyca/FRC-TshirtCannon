package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class SimpleAuto extends CommandBase {
    DriveSubsystem m_drive;
    IntakeSubsystem m_intake;
    FeederSubsystem m_feeder;
    ConveyorSubsystem m_conveyor;
    TurretSubsystem m_turret;
    LimelightSubsystem m_camera;
    ShooterSubsystem m_shooter;

    public SimpleAuto(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake, ConveyorSubsystem conveyor, FeederSubsystem feeder, 
            TurretSubsystem turret, LimelightSubsystem camera) {
        m_drive = drive;
        m_intake = intake;
        m_shooter = shooter;
        m_conveyor = conveyor;
        m_turret = turret;
        m_camera = camera;
        m_feeder = feeder;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_intake.drop(IntakeConstants.kIntakeDropSpeed);
        m_shooter.shoot(ShooterConstants.shooterSpeed);
        for (int i = 0; i < 50; i++) { //1 sec delay
            m_shooter.updatePid();
            m_turret.rotate(m_camera.getTurnSpeed());
            Robot.wait(20);
        }
        m_intake.drop(0);
        m_intake.succ(IntakeConstants.kIntakeSpeed);
        m_conveyor.convey(ConveyorConstants.kConveyorSpeed);
        m_drive.mecanumDrive(-0.3, 0, 0);
        for (int i = 0; i < 100; i++) { //2 sec delay
            m_shooter.updatePid();
            m_turret.rotate(m_camera.getTurnSpeed());
            Robot.wait(20);
        }
        m_drive.mecanumDrive(0, 0, 0);
        for (int i = 0; i < 50; i++) {
            m_shooter.updatePid();
            m_turret.rotate(m_camera.getTurnSpeed());
            Robot.wait(20);
        }
        m_drive.mecanumDrive(0.3, 0, 0);
        for (int i = 0; i < 70; i++) { //1.4 sec delay
            m_shooter.updatePid();
            m_turret.rotate(m_camera.getTurnSpeed());
            Robot.wait(20);
        }
        m_drive.mecanumDrive(0, 0, 0);
        m_feeder.feed(FeederConstants.kFeederSpeed);
        for (int i = 0; i < 200; i++) { //4 sec delay
            m_shooter.updatePid();
            m_turret.rotate(m_camera.getTurnSpeed());
            Robot.wait(20);
        }
        m_feeder.stop();
        m_conveyor.stop();
        m_intake.stop();
        m_shooter.shoot(0);
        m_shooter.updatePid();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    
}
