package frc.robot.commands;

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
import frc.robot.subsystems.ShooterSubsystem;

public class SimpleAuto extends CommandBase {
    DriveSubsystem m_drive;
    IntakeSubsystem m_intake;
    FeederSubsystem m_feeder;
    ConveyorSubsystem m_conveyor;
    ShooterSubsystem m_shooter;

    public SimpleAuto(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake, ConveyorSubsystem conveyor, FeederSubsystem feeder) {
        m_drive = drive;
        m_intake = intake;
        m_shooter = shooter;
        m_conveyor = conveyor;
        m_feeder = feeder;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_intake.drop(IntakeConstants.kIntakeDropSpeed);
        m_shooter.shoot(ShooterConstants.shooterSpeed);
        for (int i = 0; i < 50; i++) {
            m_shooter.updatePid();
            Robot.wait(20);
        }
        m_intake.drop(0);
        m_intake.succ(IntakeConstants.kIntakeSpeed);
        m_conveyor.convey(ConveyorConstants.kConveyorSpeed);
        m_drive.mecanumDrive(-0.3, 0, 0);
        Robot.wait(2000);
        m_drive.mecanumDrive(0, 0, 0);
        Robot.wait(1000);
        m_drive.mecanumDrive(0.3, 0, 0);
        Robot.wait(1400);
        m_drive.mecanumDrive(0, 0, 0);
        m_feeder.feed(FeederConstants.kFeederSpeed);
        Robot.wait(4000);
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
