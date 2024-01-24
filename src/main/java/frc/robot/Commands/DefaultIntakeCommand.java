package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends Command{
    private IntakeSubsystem m_intakeSubsystem;

    private DoubleSupplier m_pivotVelocity;
    private BooleanSupplier m_intakeNote;

    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier pivotVelocity, BooleanSupplier intakeNote) {
        m_intakeSubsystem = intakeSubsystem;
        m_pivotVelocity = pivotVelocity;
        m_intakeNote = intakeNote;

        addRequirements(m_intakeSubsystem);
    }

    public void execute() {
        m_intakeSubsystem.rotate(m_pivotVelocity.getAsDouble());

        if(m_intakeNote.getAsBoolean()) {
            m_intakeSubsystem.intake();
        } else {
            m_intakeSubsystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.rotate(0);
    }
}
