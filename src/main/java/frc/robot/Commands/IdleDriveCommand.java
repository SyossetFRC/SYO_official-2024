package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsystem;

/** A command the idles the robot drivetrain */
public class IdleDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private double m_waitTime;
    private Timer m_timer;

    /**
    * Command to idle the drivetrain indefinetely.
    *
    * @param drivetrainSubsystem The swerve drive subsystem.
    */
    public IdleDriveCommand(DrivetrainSubsystem drivetrainSubsystem) { this(drivetrainSubsystem, Double.POSITIVE_INFINITY); }

    /**
    * Command to idle the drivetrain definitely.
    *
    * @param drivetrainSubsystem The swerve drive subsystem.
    * @param waitTime The amount time to idle (s).
    */
    public IdleDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                                double waitTime) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_waitTime = waitTime;
        m_timer = new Timer();

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_drivetrainSubsystem.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() < m_waitTime) {
            return false;
        }
        return true;
    }
}