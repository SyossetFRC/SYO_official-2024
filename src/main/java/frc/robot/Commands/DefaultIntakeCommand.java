package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultIntakeCommand extends Command {
    private final IntakeSubsystem m_intakeSubsystem;

    private final DoubleSupplier m_intakeRateSupplier;
    private final DoubleSupplier m_angularSpeedSupplier;

    /**
     * Command to engage the intake using joystick input.
     * 
     * @param intakeSubsystem The intake subsystem.
     * @param intakeRateSupplier The desired intake rate (rpm).
     * @param angularSpeedSupplier The desired angle change speed (rad/s).
     */
    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier intakeRateSupplier, DoubleSupplier angularSpeedSupplier) {
        this.m_intakeSubsystem = intakeSubsystem;
        this.m_intakeRateSupplier = intakeRateSupplier;
        this.m_angularSpeedSupplier = angularSpeedSupplier;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        m_intakeSubsystem.intake(m_intakeRateSupplier.getAsDouble());
        m_intakeSubsystem.rotate(m_angularSpeedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) { 
        m_intakeSubsystem.intake(0);
        m_intakeSubsystem.rotate(0); 
    }
}