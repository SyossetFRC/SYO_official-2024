package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class AutonIntakeCommand extends Command{
    private final IntakeSubsystem m_intakeSubsystem;

    private final double m_intakeRateSupplier;
    private final double m_angle;

    private final long m_maxTime;
    private long m_recordedTime;
    private boolean m_isTimeRecorded;

    private final PIDController m_anglePIDController;

    /**
     * Command to engage the intake autonomously.
     * 
     * @param intakeSubsystem The intake subsystem.
     * @param intakeRateSupplier The desired rate (rpm).
     * @param angle The desired angle (rad).
     * @param maxTime The maximum time alloted for this command (ms).
     */
    public AutonIntakeCommand(IntakeSubsystem intakeSubsystem, double intakeRateSupplier, double angle, long maxTime) {
        this.m_intakeSubsystem = intakeSubsystem;
        this.m_intakeRateSupplier = intakeRateSupplier;
        this.m_angle = angle;

        this.m_maxTime = maxTime;
        this.m_isTimeRecorded = false;

        m_anglePIDController = new PIDController(2.0, 0.1, 0.1);
        m_anglePIDController.setTolerance(0.10);

        addRequirements(intakeSubsystem);
    }

    /**
     * Command to engage the intake autonomously. The intake will not rotate.
     * 
     * @param intakeSubsystem The intake subsystem.
     * @param intakeRateSupplier The desired rate (rpm).
     * @param maxTime The maximum time alloted for this command (ms).
     */
    public AutonIntakeCommand(IntakeSubsystem intakeSubsystem, double intakeRateSupplier, long maxTime) {
        this(intakeSubsystem, intakeRateSupplier, intakeSubsystem.getAngle(), maxTime);
    }

    @Override
    public void execute() {
        if (!m_isTimeRecorded) {
            m_recordedTime = System.currentTimeMillis();
            m_isTimeRecorded = true;
        }

        m_intakeSubsystem.intake(m_intakeRateSupplier);
        m_intakeSubsystem.rotate(m_anglePIDController.calculate(m_intakeSubsystem.getAngle(), m_angle));
    }

    @Override
    public boolean isFinished() { return System.currentTimeMillis() > m_recordedTime + m_maxTime; }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.intake(0);
        m_intakeSubsystem.rotate(0);
    }
}