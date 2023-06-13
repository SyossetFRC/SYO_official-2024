package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

public class LimelightAlignmentDriveCommand extends CommandBase {
    private final LimelightSubsystem m_limelightSubsystem;
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private PIDController m_yPIDController;
    private PIDController m_xPIDController;
    private PIDController m_angController;

    private double m_yVelocity;
    private double m_xVelocity;
    private double m_angularVelocity;
    private double m_distanceToRobot;
    private double m_deadband;
    private double m_kP;
    private double m_isFinishedTolerance;
    private String m_yControlMode;

    /**
     * @param drivetrainSubsystem The robot's drivetrain subsystem
     * @param limelightSubsystem  The robot's Limelight subsystem
     * @param distanceToRobot     The desired distance to maintain from the player
     * @param yControlMode        The type of correctional movement in the y
     *                            direction: "translational" or "rotational"
     */
    public LimelightAlignmentDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
            LimelightSubsystem limelightSubsystem, String yControlMode) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_limelightSubsystem = limelightSubsystem;

        m_distanceToRobot = 1;
        m_isFinishedTolerance = 0.05;
        m_kP = 0.01;
        m_deadband = 0.005;
        m_yControlMode = yControlMode;

        addRequirements(m_drivetrainSubsystem, m_limelightSubsystem);
    }

    public void initialize() {
        m_yPIDController = new PIDController(m_kP, 0, 0);
        m_xPIDController = new PIDController(m_kP, 0, 0);
        m_angController = new PIDController(m_kP, 0, 0);
    }

    public void execute() {
        if (m_yControlMode.equals("translational")) {
            m_yVelocity = m_yPIDController.calculate(m_limelightSubsystem.getXTargetAngle());
        } else {
            m_angularVelocity = m_angController.calculate(m_limelightSubsystem.getXTargetAngle());
        }

        m_xVelocity = -m_xPIDController.calculate((m_limelightSubsystem.getDistance() - m_distanceToRobot) * 100);

        m_xVelocity = (Math.abs(m_xVelocity) < m_deadband) ? 0 : m_xVelocity;
        m_yVelocity = (Math.abs(m_yVelocity) < m_deadband) ? 0 : m_yVelocity;
        m_angularVelocity = (Math.abs(m_angularVelocity) < m_deadband * 15) ? 0 : m_angularVelocity;
        if (m_limelightSubsystem.getFoundTag()) {
            m_drivetrainSubsystem.drive(
                    m_xVelocity,
                    m_yVelocity,
                    m_angularVelocity*1.3,
                    false);
        }
        else {
            m_drivetrainSubsystem.drive(0, 0, 0, true);
        }
    }

    public boolean isFinished() {
        if (Math.abs(m_limelightSubsystem.getXTargetAngle()) > m_isFinishedTolerance || Math.abs(m_limelightSubsystem.getDistance() - m_distanceToRobot) > 0.1) {
            return false;
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0, true);
    }
}