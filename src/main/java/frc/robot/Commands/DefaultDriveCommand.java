package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    /**
    * Command to drive the robot using joystick input.
    *
    * @param drivetrainSubsystem The swerve drive subsystem.
    * @param translationXSupplier The x translational velocity (m/s).
    * @param translationYSupplier The y translational velocity (m/s).
    * @param rotationSupplier The rotational velocity (rad/s).
    */
    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.drive(
                m_translationXSupplier.getAsDouble(),
                m_translationYSupplier.getAsDouble(),
                m_rotationSupplier.getAsDouble(),
                true
        );
    }

    @Override
    public void end(boolean interrupted) { m_drivetrainSubsystem.drive(0, 0, 0, false); }
}
