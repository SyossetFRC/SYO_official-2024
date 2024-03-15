package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

public class LimelightRotateCommand extends Command {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final LimelightSubsystem m_limelightSubsysystem;

    private final double m_rotationalVelocity;

    private double m_rotationSupplier;

    private double m_outputTheta;

    private final long m_maxTime;
    private long m_recordedTime;
    private boolean m_isTimeRecorded;

    /**
    * Command to rotate the robot to face the speaker using limelight input.
    *
    * @param drivetrainSubsystem The swerve drive subsystem.
    * @param limelightSubsystem The limelight subsystem.
    * @param rotationalVelocity The theoretical rotational velocity (rad/s).
    * @param maxTime The maximum time allowed to elapse (ms).
    */
    public LimelightRotateCommand(DrivetrainSubsystem drivetrainSubsystem,
                                LimelightSubsystem limelightSubsystem,
                                double rotationalVelocity,
                                long maxTime) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_limelightSubsysystem = limelightSubsystem;

        m_rotationalVelocity = rotationalVelocity;

        m_maxTime = maxTime;

        m_isTimeRecorded = false;

        addRequirements(m_drivetrainSubsystem);
    }

    /**
    * Command to rotate the robot to face the speaker using limelight input. Theoretical velocity is at maximum.
    *
    * @param drivetrainSubsystem The swerve drive subsystem.
    * @param limelightSubsystem The limelight subsystem.
    * @param maxTime The maximum time allowed to elapse (ms).
    */
    public LimelightRotateCommand(DrivetrainSubsystem drivetrainSubsystem,
                                LimelightSubsystem limelightSubsystem,
                                long maxTime) {
        this(drivetrainSubsystem, limelightSubsystem, DrivetrainSubsystem.kMaxAngularSpeed, maxTime);
    }



    
    @Override
    public void initialize() { m_rotationSupplier = Math.copySign(m_rotationalVelocity, m_limelightSubsysystem.getDrivetrainAngleChange()); }
    
    @Override
    public void execute() {
        if (!m_isTimeRecorded) {
            m_recordedTime = System.currentTimeMillis();
            m_isTimeRecorded = true;
        }

        m_outputTheta = clip(m_limelightSubsysystem.getDrivetrainAngleChange(), m_rotationSupplier);

        m_drivetrainSubsystem.drive(
                0.01,
                0,
                m_outputTheta,
                true
        );
    }

    @Override
    public boolean isFinished() { return System.currentTimeMillis() > m_recordedTime + m_maxTime; }

    @Override
    public void end(boolean interrupted) { 
        m_drivetrainSubsystem.drive(0, 0, 0, true); 
        m_isTimeRecorded = false;
    }

    /**
     * Clips a certain value if it exceeds a certain range.
     * 
     * @param value Value to clip.
     * @param range Defines the range [-range, range].
     * @return Returns the value if not clipped and either -range or range if clipped.
     */
    private double clip(double value, double range) {
        value = Math.min(value, Math.abs(range));
        value = Math.max(value, -Math.abs(range));
        return value;
    }
}