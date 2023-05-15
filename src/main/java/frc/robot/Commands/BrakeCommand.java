package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class BrakeCommand extends CommandBase {
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
                0,
                0,
                0.0001,
                true
        );
    }

    @Override
    public void end(boolean interrupted) { m_drivetrainSubsystem.drive(0, 0, 0, true); }
}
