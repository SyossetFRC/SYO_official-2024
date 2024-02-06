package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OuttakeSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultOuttakeCommand extends Command {
    private final OuttakeSubsystem m_outtakeSubsystem;

    private final DoubleSupplier m_outtakeRateSupplier;

    /**
     * Command to engage the outtake using joystick input.
     * 
     * @param outtakeSubsystem The outtake subsystem.
     * @param outtakeRateSupplier The desired outtake rate (rpm).
     */
    public DefaultOuttakeCommand(OuttakeSubsystem outtakeSubsystem, DoubleSupplier outtakeRateSupplier) {
        this.m_outtakeSubsystem = outtakeSubsystem;
        this.m_outtakeRateSupplier = outtakeRateSupplier;

        addRequirements(outtakeSubsystem);
    }

    @Override
    public void execute() {
        m_outtakeSubsystem.outtake(m_outtakeRateSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) { 
        m_outtakeSubsystem.outtake(0);
    }
}