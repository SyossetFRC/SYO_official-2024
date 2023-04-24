package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    private Pose2d m_initialPose;
    private Rotation2d m_initialAngle;

    private boolean m_isXFinished = false;
    private boolean m_isYFinished = false;
    private boolean m_isThetaFinished = false;

    /**
    * Method to drive the robot using joystick info.
    *
    * @param x X coordinate to move to (m).
    * @param y Y coordinate to move to (m).
    * @param theta Angle to rotate to (rad).
    * @param translationalVelocity Translational velocity (m/s).
    * @param rotationalVelocity Rotational velocity (rad/s).
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
        m_initialPose = m_drivetrainSubsystem.getPose();
        m_initialAngle = m_drivetrainSubsystem.getAngle();

        double distanceX = m_x - m_initialPose.getX();
        double distanceY = m_y - m_initialPose.getY();

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

        if (Math.abs(m_drivetrainSubsystem.getPose().getX() - m_x) < 5) {
            m_translationXSupplier = () -> 0;
            m_isXFinished = true;
        }
        if (Math.abs(m_drivetrainSubsystem.getPose().getY() - m_y) < 5) {
            m_translationYSupplier = () -> 0;
            m_isYFinished = true;
        }
        if (Math.abs(m_drivetrainSubsystem.getAngle().getRadians() - m_theta) < Math.PI / 60) {
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
