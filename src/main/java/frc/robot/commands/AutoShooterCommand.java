package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoShooterCommand extends CommandBase {
    ShooterSubsystem m_shooter;
    ConveyorSubsystem m_conveyor;
    FeederSubsystem m_feeder;
    BlinkinSubsystem m_led;
    TurretSubsystem m_turret;
    LimelightSubsystem m_camera;

    public AutoShooterCommand(ShooterSubsystem shooter, ConveyorSubsystem conveyor, FeederSubsystem feeder,
            BlinkinSubsystem led, TurretSubsystem turret, LimelightSubsystem camera) {
        m_shooter = shooter;
        m_conveyor = conveyor;
        m_feeder = feeder;
        m_turret = turret;
        m_led = led;
        m_camera = camera;
        addRequirements(m_shooter, m_conveyor, m_feeder, m_led, m_camera);
    }

    @Override
    public void execute() {
        m_shooter.shoot((ShooterConstants.shooterSpeed * m_camera.getShooterSpeed()));
        for (int i = 0; i < 100; i++) { // 2 sec delay
            m_shooter.updatePid();
            m_turret.rotate(m_camera.getTurnSpeed());
            Robot.wait(20);
        }
        m_led.green();
        m_feeder.feed(FeederConstants.kFeederSpeed);
        for (int i = 0; i < 100; i++) { // 2 sec delay
            m_shooter.updatePid();
            m_turret.rotate(m_camera.getTurnSpeed());
            Robot.wait(20);
        }
        m_conveyor.convey(ConveyorConstants.kConveyorSpeed);
        m_feeder.feed(FeederConstants.kFeederSpeed);
        for (int i = 0; i < 100; i++) { // 2 sec delay
            m_shooter.updatePid();
            m_turret.rotate(m_camera.getTurnSpeed());
            Robot.wait(20);
        }
        m_feeder.stop();
        m_conveyor.stop();
        m_led.red();
        m_shooter.shoot(0);
        m_shooter.updatePid();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
