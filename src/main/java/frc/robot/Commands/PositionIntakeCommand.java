package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class PositionIntakeCommand extends Command{
    private final IntakeSubsystem m_intakeSubsystem;

    private final DoubleSupplier m_intakeRateSupplier;
    private final DoubleSupplier m_angle;

    private final PIDController m_anglePIDController;

    /**
     * Command to engage the intake using joystick input.
     * 
     * @param intakeSubsystem The intake subsystem.
     * @param intakeRateSupplier The desired rate (rpm).
     * @param angle The desired angle (rad).
     */
    public PositionIntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier intakeRateSupplier, DoubleSupplier angle) {
        this.m_intakeSubsystem = intakeSubsystem;
        this.m_intakeRateSupplier = intakeRateSupplier;
        this.m_angle = angle;

        m_anglePIDController = new PIDController(2.0, 0.2, 0.1);
        m_anglePIDController.setTolerance(0.03);

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        m_intakeSubsystem.intake(m_intakeRateSupplier.getAsDouble());
        m_intakeSubsystem.rotate(m_anglePIDController.calculate(m_intakeSubsystem.getAngle(), m_angle.getAsDouble()));
    }

    @Override
    public boolean isFinished() { return m_anglePIDController.atSetpoint(); }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.intake(0);
        m_intakeSubsystem.rotate(0);
    }
}