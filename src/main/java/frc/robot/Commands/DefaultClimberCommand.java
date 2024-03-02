package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimberSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultClimberCommand extends Command {
    private final ClimberSubsystem m_climberSubsystem;

    private final DoubleSupplier m_climberPowerLeftSupplier;
    private final DoubleSupplier m_climberPowerRightSupplier;

    /**
     * Command to engage the climber using joystick input.
     * 
     * @param climberSubsystem The climber subsystem.
     * @param climberPowerLeftSupplier The desired left climber power [-1, 1].
     * @param climberPowerRightSupplier The desired right climber power [-1, 1].
     */
    public DefaultClimberCommand(ClimberSubsystem climberSubsystem, DoubleSupplier climberPowerLeftSupplier, DoubleSupplier climberPowerRightSupplier) {
        this.m_climberSubsystem = climberSubsystem;
        this.m_climberPowerLeftSupplier = climberPowerLeftSupplier;
        this.m_climberPowerRightSupplier = climberPowerRightSupplier;

        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        m_climberSubsystem.climb(m_climberPowerLeftSupplier.getAsDouble(), m_climberPowerRightSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) { 
        m_climberSubsystem.climb(0, 0);
    }
}