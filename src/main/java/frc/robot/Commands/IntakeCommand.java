package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeCommand extends Command{
    private final IntakeSubsystem m_intakeSubsystem;

    private final DoubleSupplier m_intakeRateSupplier;
    private final double m_waitTime;

    private long m_recordedTime;
    private boolean m_isTimeRecorded;

    /**
     * Command to engage the intake using joystick input.
     * 
     * @param intakeSubsystem The intake subsystem.
     * @param intakeRateSupplier The desired rate (rpm).
     * @param waitTime Time to spin intake (s).
     */
    public IntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier intakeRateSupplier, double waitTime) {
        this.m_intakeSubsystem = intakeSubsystem;
        this.m_intakeRateSupplier = intakeRateSupplier;
        this.m_waitTime = waitTime;

        m_isTimeRecorded = false;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (!m_isTimeRecorded) {
            m_recordedTime = System.currentTimeMillis();
            m_isTimeRecorded = true;
        }

        m_intakeSubsystem.intake(m_intakeRateSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() { return System.currentTimeMillis() > m_recordedTime + (m_waitTime * 1000); }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.intake(0);
    }
}