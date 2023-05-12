package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class PositionDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final double m_x;
    private final double m_y;
    private final double m_theta;

    private final double m_translationalVelocity;
    private final double m_rotationalVelocity;

    private DoubleSupplier m_translationXSupplier;
    private DoubleSupplier m_translationYSupplier;
    private DoubleSupplier m_rotationSupplier;

    private Translation2d m_initialPosition;
    private Rotation2d m_initialAngle;

    private boolean m_isXFinished = false;
    private boolean m_isYFinished = false;
    private boolean m_isThetaFinished = false;

    /**
    * Method to drive the robot autonomously.
    *
    * @param drivetrainSubsystem The swerve drive subsystem.
    * @param x The x coordinate to move to (m).
    * @param y The y coordinate to move to (m).
    * @param theta The angle to rotate to (rad).
    * @param translationalVelocity The resultant translational velocity (m/s).
    * @param rotationalVelocity The rotational velocity (rad/s).
    */
    public PositionDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                                double x,
                                double y,
                                double theta,
                                double translationalVelocity,
                                double rotationalVelocity) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_x = x;
        m_y = y;
        m_theta = theta;

        m_translationalVelocity = translationalVelocity;
        m_rotationalVelocity = rotationalVelocity;

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        m_initialPosition = m_drivetrainSubsystem.getPosition();
        m_initialAngle = m_drivetrainSubsystem.getAngle();

        double distanceX = m_x - m_initialPosition.getX();
        double distanceY = m_y - m_initialPosition.getY();

        m_translationXSupplier = () -> (distanceX / Math.hypot(distanceX, distanceY) * m_translationalVelocity);
        m_translationYSupplier = () -> (distanceY / Math.hypot(distanceX, distanceY) * m_translationalVelocity);
        m_rotationSupplier = () -> Math.copySign(m_rotationalVelocity, m_theta - m_initialAngle.getRadians());
    }
    
    @Override
    public void execute() {
        m_drivetrainSubsystem.drive(
                m_translationXSupplier.getAsDouble(),
                m_translationYSupplier.getAsDouble(),
                m_rotationSupplier.getAsDouble(),
                true
        );

        if (Math.abs(m_drivetrainSubsystem.getPosition().getX() - m_x) < 0.1) {
            m_translationXSupplier = () -> 0;
            m_isXFinished = true;
        }
        if (Math.abs(m_drivetrainSubsystem.getPosition().getY() - m_y) < 0.1) {
            m_translationYSupplier = () -> 0;
            m_isYFinished = true;
        }
        if (Math.abs(m_drivetrainSubsystem.getAngle().getRadians() - m_theta) < (Math.PI / 40)) {
            m_rotationSupplier = () -> 0;
            m_isThetaFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_isXFinished && m_isYFinished && m_isThetaFinished;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0, false);
    }
}
