package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OuttakeSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultOuttakeCommand extends Command {
    private final OuttakeSubsystem m_outtakeSubsystem;

    private final DoubleSupplier m_outtakeRateSupplier;
    private final DoubleSupplier m_outtakeRotatePowerSupplier;

    /**
     * Command to engage the outtake using joystick input.
     * 
     * @param outtakeSubsystem The outtake subsystem.
     * @param outtakeRateSupplier The desired outtake rate (rpm).
     * @param outtakeRotatePowerSupplier The desired outtake rotate power [-1.0, 1.0].
     */
    public DefaultOuttakeCommand(OuttakeSubsystem outtakeSubsystem, DoubleSupplier outtakeRateSupplier, DoubleSupplier outtakeRotatePowerSupplier) {
        this.m_outtakeSubsystem = outtakeSubsystem;
        this.m_outtakeRateSupplier = outtakeRateSupplier;
        this.m_outtakeRotatePowerSupplier = outtakeRotatePowerSupplier;

        addRequirements(outtakeSubsystem);
    }

    @Override
    public void execute() {
        m_outtakeSubsystem.outtake(m_outtakeRateSupplier.getAsDouble());
        m_outtakeSubsystem.rotate(m_outtakeRotatePowerSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) { 
        m_outtakeSubsystem.outtake(0);
        m_outtakeSubsystem.rotate(0);
    }
}