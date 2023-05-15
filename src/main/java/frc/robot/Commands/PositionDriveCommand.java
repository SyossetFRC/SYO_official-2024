package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
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

    private final double m_deccelDistance;
    private final double m_deccelTheta;

    private double m_translationXSupplier;
    private double m_translationYSupplier;
    private double m_rotationSupplier;

    private PIDController m_pidX;
    private PIDController m_pidY;
    private PIDController m_pidTheta;

    private Translation2d m_initialPosition;
    private Rotation2d m_initialAngle;

    private boolean m_isFinishedX;
    private boolean m_isFinishedY;
    private boolean m_isFinishedTheta;

    private double m_outputX;
    private double m_outputY;
    private double m_outputTheta;

    /**
    * Command to drive the robot autonomously.
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

        m_deccelDistance = 2.0;
        m_deccelTheta = Math.toRadians(120);

        addRequirements(m_drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        m_initialPosition = m_drivetrainSubsystem.getPosition();
        m_initialAngle = m_drivetrainSubsystem.getAngle();

        double distanceX = m_x - m_initialPosition.getX();
        double distanceY = m_y - m_initialPosition.getY();

        m_translationXSupplier = (distanceX / Math.hypot(distanceX, distanceY)) * m_translationalVelocity;
        m_translationYSupplier = (distanceY / Math.hypot(distanceX, distanceY)) * m_translationalVelocity;
        m_rotationSupplier = Math.copySign(m_rotationalVelocity, m_theta - m_initialAngle.getRadians());

        m_pidX = new PIDController(-m_translationXSupplier / m_deccelDistance, 0, 0);
        m_pidY = new PIDController(-m_translationYSupplier / m_deccelDistance, 0, 0);
        m_pidTheta = new PIDController(-m_rotationSupplier / m_deccelTheta, 0, 0);

        m_pidX.setTolerance(0.01);
        m_pidY.setTolerance(0.01);
        m_pidTheta.setTolerance(Math.toRadians(1));

        m_isFinishedX = false;
        m_isFinishedY = false;
        m_isFinishedTheta = false;
    }
    
    @Override
    public void execute() {
        double errorX = Math.abs(m_drivetrainSubsystem.getPosition().getX() - m_x);
        double errorY = Math.abs(m_drivetrainSubsystem.getPosition().getY() - m_y);
        double errorTheta = Math.abs(m_drivetrainSubsystem.getAngle().getRadians() - m_theta);

        if (!m_isFinishedX) { m_outputX = m_pidX.calculate(Math.min(errorX, m_deccelDistance), 0); }
        if (m_pidX.atSetpoint()) { 
            m_outputX = 0; 
            m_isFinishedX = true; 
        }

        if (!m_isFinishedY) { m_outputY = m_pidY.calculate(Math.min(errorY, m_deccelDistance), 0); }
        if (m_pidY.atSetpoint()) { 
            m_outputY = 0; 
            m_isFinishedY = true; 
        }

        if (!m_isFinishedTheta) { m_outputTheta = m_pidTheta.calculate(Math.min(errorTheta, m_deccelTheta), 0); }
        if (m_pidTheta.atSetpoint()) { 
            m_outputTheta = 0; 
            m_isFinishedTheta = true; 
        }

        m_drivetrainSubsystem.drive(
                m_outputX,
                m_outputY,
                m_outputTheta,
                true
        );
    }

    @Override
    public boolean isFinished() { return (m_outputX == 0) && (m_outputY == 0) && (m_outputTheta == 0); }

    @Override
    public void end(boolean interrupted) { m_drivetrainSubsystem.drive(0, 0, 0, true); }
}
