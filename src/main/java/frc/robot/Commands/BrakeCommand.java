package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSubsystem;

/** A command that brakes the drivetrain using swerve module alignment. */
public class BrakeCommand extends Command {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    /**
    * Command to brake the drivetrain.
    *
    * @param drivetrainSubsystem The swerve drive subsystem.
    */
    public BrakeCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        m_drivetrainSubsystem.drive(
                0.001,
                0,
                0.001,
                true
        );
    }

    @Override
    public void end(boolean interrupted) { m_drivetrainSubsystem.drive(0, 0, 0, true); }
}
