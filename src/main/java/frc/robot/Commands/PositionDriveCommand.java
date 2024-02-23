package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSubsystem;

/** An autonomous command that translates and rotates the robot to any absolute position */
public class PositionDriveCommand extends Command {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final double m_x;
    private final double m_y;
    private final double m_theta;

    private final double m_translationalVelocity;
    private final double m_rotationalVelocity;

    private double m_translationXSupplier;
    private double m_translationYSupplier;
    private double m_rotationSupplier;

    private PIDController m_pidX;
    private PIDController m_pidY;
    private PIDController m_pidTheta;

    private Translation2d m_initialPosition;
    private Rotation2d m_initialAngle;

    private double m_outputX;
    private double m_outputY;
    private double m_outputTheta;

    private final long m_maxTime;
    private long m_recordedTime;
    private boolean m_isTimeRecorded;

    /**
    * Command to drive the robot autonomously.
    *
    * @param drivetrainSubsystem The swerve drive subsystem.
    * @param x The x coordinate to move to (m).
    * @param y The y coordinate to move to (m).
    * @param theta The angle to rotate to (rad).
    * @param translationalVelocity The resultant theoretical translational velocity (m/s).
    * @param rotationalVelocity The theoretical rotational velocity (rad/s).
    * @param maxTime The maximum time allowed to elapse (ms).
    */
    public PositionDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                                double x,
                                double y,
                                double theta,
                                double translationalVelocity,
                                double rotationalVelocity,
                                long maxTime) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        
        if (x == 0 && y == 0) {
            m_x = 0.001;
        }
        else {
            m_x = x;
        }
        m_y = y;
        m_theta = theta;

        m_translationalVelocity = translationalVelocity;
        m_rotationalVelocity = rotationalVelocity;

        m_maxTime = maxTime;

        m_isTimeRecorded = false;

        addRequirements(m_drivetrainSubsystem);
    }

    /**
    * Command to drive the robot autonomously. Theoretical velocity is at a maximum.
    *
    * @param drivetrainSubsystem The swerve drive subsystem.
    * @param x The x coordinate to move to (m).
    * @param y The y coordinate to move to (m).
    * @param theta The angle to rotate to (rad).
    * @param maxTime The maximum time allowed to elapse (ms).
    */
    public PositionDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                                double x,
                                double y,
                                double theta,
                                long maxTime) {
        this(drivetrainSubsystem, x, y, theta, DrivetrainSubsystem.kMaxSpeed, DrivetrainSubsystem.kMaxAngularSpeed, maxTime);
    }

    @Override
    public void initialize() {
        // Records initial Pose
        m_initialPosition = m_drivetrainSubsystem.getPosition();
        m_initialAngle = m_drivetrainSubsystem.getAngle();

        // Calculates needed displacement
        double distanceX = m_x - m_initialPosition.getX();
        double distanceY = m_y - m_initialPosition.getY();

        // Calculates theoretical movement vectors
        m_translationXSupplier = (distanceX / Math.hypot(distanceX, distanceY)) * m_translationalVelocity;
        m_translationYSupplier = (distanceY / Math.hypot(distanceX, distanceY)) * m_translationalVelocity;
        m_rotationSupplier = Math.copySign(m_rotationalVelocity, m_theta - m_initialAngle.getRadians());

        // Creates velocity component PIDs
        m_pidX = new PIDController(2.0, 0.1, 0.1);
        m_pidY = new PIDController(2.0, 0.1, 0.1);
        m_pidTheta = new PIDController(2.5, 0.2, 0.1);

        m_pidX.setIZone(1.0);
        m_pidY.setIZone(1.0);
        m_pidTheta.setIZone(Math.PI / 2);

        m_pidX.setTolerance(0.05);
        m_pidY.setTolerance(0.05);
        m_pidTheta.setTolerance(0.05);

        m_pidTheta.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    @Override
    public void execute() {
        if (!m_isTimeRecorded) {
            m_recordedTime = System.currentTimeMillis();
            m_isTimeRecorded = true;
        }

        m_outputX = clip(m_pidX.calculate(m_drivetrainSubsystem.getPosition().getX(), m_x), m_translationXSupplier);
        m_outputY = clip(m_pidY.calculate(m_drivetrainSubsystem.getPosition().getY(), m_y), m_translationYSupplier);
        m_outputTheta = clip(m_pidTheta.calculate(m_drivetrainSubsystem.getAngle().getRadians(), m_theta), m_rotationSupplier);

        m_drivetrainSubsystem.drive(
                m_outputX,
                m_outputY,
                m_outputTheta,
                true
        );
    }

    @Override
    public boolean isFinished() { return (m_pidX.atSetpoint() && m_pidY.atSetpoint() && m_pidTheta.atSetpoint()) || (System.currentTimeMillis() > m_recordedTime + m_maxTime); }

    @Override
    public void end(boolean interrupted) { m_drivetrainSubsystem.drive(0, 0, 0, true); }

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
