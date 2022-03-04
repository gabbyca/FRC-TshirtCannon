package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShooterCommand extends CommandBase {
    ShooterSubsystem m_shooter;
    ConveyorSubsystem m_conveyor;
    FeederSubsystem m_feeder;
    BlinkinSubsystem m_led;
    LimelightSubsystem m_camera;
    private final double m_speed;

    public AutoShooterCommand(ShooterSubsystem shooter, ConveyorSubsystem conveyor, FeederSubsystem feeder,
            BlinkinSubsystem led, LimelightSubsystem camera, double speed) {
        m_shooter = shooter;
        m_conveyor = conveyor;
        m_feeder = feeder;
        m_led = led;
        m_camera = camera;
        m_speed = speed;
        addRequirements(m_shooter, m_conveyor, m_feeder, m_led, m_camera);
    }

    @Override
    public void execute() {
        m_shooter.shoot(ShooterConstants.shooterSpeed);
        m_led.green();
        m_shooter.shoot(m_speed * m_camera.getShooterSpeed());
        m_feeder.feed(FeederConstants.kFeederSpeed);
        Robot.wait(500);
        m_conveyor.convey(ConveyorConstants.kConveyorSpeed);
        Robot.wait(500);
        m_conveyor.stop();
        Robot.wait(1000);
        m_feeder.stop();
        m_led.red();
        m_shooter.shoot(ShooterConstants.shooterSpeed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
