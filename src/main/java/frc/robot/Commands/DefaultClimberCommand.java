package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimberSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultClimberCommand extends Command {
    private final ClimberSubsystem m_climberSubsystem;

    private final DoubleSupplier m_climberPowerSupplier;

    /**
     * Command to engage the climber using joystick input.
     * 
     * @param climberSubsystem The climber subsystem.
     * @param climberPowerSupplier The desired climber power [-1, 1].
     */
    public DefaultClimberCommand(ClimberSubsystem climberSubsystem, DoubleSupplier climberPowerSupplier) {
        this.m_climberSubsystem = climberSubsystem;
        this.m_climberPowerSupplier = climberPowerSupplier;

        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        m_climberSubsystem.climb(m_climberPowerSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) { 
        m_climberSubsystem.climb(0);
    }
}