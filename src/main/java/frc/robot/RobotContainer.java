package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.BrakeCommand;
import frc.robot.Commands.DefaultDriveCommand;
import frc.robot.Commands.DefaultIntakeCommand;
import frc.robot.Commands.IdleDriveCommand;
import frc.robot.Commands.PositionDriveCommand;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

/** Represents the entire robot. */
public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  // private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final Joystick m_driveController = new Joystick(0);
  // private final Joystick m_operatorController = new Joystick(1);
  private double m_powerLimit = 1.0;

  /**
   * This class stores all robot related subsystems, commands, and methods that
   * the {@link Robot} class can utilize during different OpModes.
   */
  public RobotContainer() {
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        m_drivetrainSubsystem,
        () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(1), 0.05) * m_powerLimit
            * DrivetrainSubsystem.kMaxSpeed,
        () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(0), 0.05) * m_powerLimit
            * DrivetrainSubsystem.kMaxSpeed,
        () -> (-MathUtil.applyDeadband(m_driveController.getRawAxis(4), 0.05) / 2.0) * m_powerLimit
            * DrivetrainSubsystem.kMaxAngularSpeed
    ));

    /*
    m_intakeSubsystem.setDefaultCommand(new DefaultIntakeCommand(
        m_intakeSubsystem, 
        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(1), 0.05) * IntakeSubsystem.kIntakeMaxRate, 
        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(3), 0.05) * IntakeSubsystem.kRotateMaxAngularSpeed
    ));
    */

    configureButtons();
  }

  // Currently used for testing kinematics
  public Command autonomousCommands() {
    m_powerLimit = 1.0;
    // m_intakeSubsystem.reset();
    return new SequentialCommandGroup(
      new PositionDriveCommand(m_drivetrainSubsystem, 1.0, 0.5, Math.PI / 2, 2.5, Math.PI),
      new PositionDriveCommand(m_drivetrainSubsystem, 2.0, 0, 0, 2.5, Math.PI),
      new PositionDriveCommand(m_drivetrainSubsystem, 1.0, -0.5, -Math.PI / 2, 2.5, Math.PI),
      new PositionDriveCommand(m_drivetrainSubsystem, 0, 0, 0, 2.5, Math.PI)
    );
  }

  private void configureButtons() {
    // Driver button A
    Trigger m_resetPose = new Trigger(() -> m_driveController.getRawButton(1));
    m_resetPose.onTrue(new InstantCommand(() -> setPose(0, 0, 0)));

    // Driver button X
    Trigger m_brake = new Trigger(() -> m_driveController.getRawButton(3));
    m_brake.onTrue(new BrakeCommand(m_drivetrainSubsystem));
    m_brake.onFalse(new InstantCommand(() -> m_drivetrainSubsystem.getCurrentCommand().cancel()));

    // Driver D-pad up
    Trigger m_incrementPowerLimit = new Trigger(() -> (m_driveController.getPOV() >= 315
        || (m_driveController.getPOV() <= 45 && m_driveController.getPOV() >= 0)));
    m_incrementPowerLimit.onTrue(new InstantCommand(() -> changePowerLimit(0.2)));

    // Driver D-pad down
    Trigger m_decrementPowerLimit = new Trigger(
        () -> (m_driveController.getPOV() >= 135 && m_driveController.getPOV() <= 225));
    m_decrementPowerLimit.onTrue(new InstantCommand(() -> changePowerLimit(-0.2)));
  }

  public void setPose(double xPos, double yPos, double theta) {
    m_drivetrainSubsystem.setPose(xPos, yPos, theta);
    m_drivetrainSubsystem.alignTurningEncoders();
  }

  public void setIdleMode(String idleMode) {
    m_drivetrainSubsystem.setIdleMode(idleMode);
  }

  private void changePowerLimit(double delta) {
    if ((m_powerLimit <= 1.0 - Math.abs(delta) || delta <= 0) && (m_powerLimit >= Math.abs(delta) || delta >= 0)) {
      m_powerLimit += delta;
    }
  }
}
