package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AutonIntakeCommand;
import frc.robot.Commands.AutonOuttakeCommand;
import frc.robot.Commands.BrakeCommand;
import frc.robot.Commands.DefaultDriveCommand;
import frc.robot.Commands.DefaultIntakeCommand;
import frc.robot.Commands.DefaultOuttakeCommand;
import frc.robot.Commands.PositionDriveCommand;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.OuttakeSubsystem;

/** Represents the entire robot. */
public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final OuttakeSubsystem m_outtakeSubsystem = new OuttakeSubsystem();

  private final Joystick m_driveController = new Joystick(0);
  private final Joystick m_operatorController = new Joystick(1);
  private final Joystick m_buttonBoard = new Joystick(2);
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
        () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(4), 0.05) * m_powerLimit
            * DrivetrainSubsystem.kMaxAngularSpeed * 0.5
    ));

    m_intakeSubsystem.setDefaultCommand(new DefaultIntakeCommand(
        m_intakeSubsystem, 
        () -> getDPadInput(m_operatorController) * IntakeSubsystem.kIntakeMaxRate * 0.15, 
        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(5), 0.05) * IntakeSubsystem.kRotateMaxAngularSpeed * 0.15
    ));

    m_outtakeSubsystem.setDefaultCommand(new DefaultOuttakeCommand(
        m_outtakeSubsystem, 
        () -> MathUtil.applyDeadband(m_operatorController.getRawAxis(3), 0.05) * OuttakeSubsystem.kOuttakeMaxRate,
        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(1), 0.05)
    ));

    configureButtons();
  }

  // Currently used for testing kinematics
  public Command autonomousCommands() {
    m_powerLimit = 1.0;
    m_drivetrainSubsystem.alignTurningEncoders();
    m_intakeSubsystem.reset();
    return new SequentialCommandGroup(
      intakeSequence(),
      new PositionDriveCommand(m_drivetrainSubsystem, -3, 0, 0, 2000),
      outtakeSpeakerSequence(0)
    );
  }

  private void configureButtons() {
    // Driver button A
    Trigger m_resetPose = new Trigger(() -> m_driveController.getRawButton(1));
    m_resetPose.onTrue(new InstantCommand(() -> setPose(0, 0, 0)));

    // Operator button A
    Trigger m_resetSubsystems = new Trigger(() -> m_operatorController.getRawButton(1));
    m_resetSubsystems.onTrue(new InstantCommand(() -> m_intakeSubsystem.reset()));

    // Driver button X
    Trigger m_brake = new Trigger(() -> m_driveController.getRawButton(3));
    m_brake.onTrue(new BrakeCommand(m_drivetrainSubsystem));
    m_brake.onFalse(new InstantCommand(() -> m_drivetrainSubsystem.getCurrentCommand().cancel()));

    // Driver D-pad up
    Trigger m_incrementPowerLimit = new Trigger(() -> getDPadInput(m_driveController) == 1.0);
    m_incrementPowerLimit.onTrue(new InstantCommand(() -> changePowerLimit(0.2)));

    // Driver D-pad down
    Trigger m_decrementPowerLimit = new Trigger(() -> getDPadInput(m_driveController) == -1.0);
    m_decrementPowerLimit.onTrue(new InstantCommand(() -> changePowerLimit(-0.2)));

    // Button board column 1, row 2
    Trigger m_intake = new Trigger(() -> m_buttonBoard.getRawButton(1));
    m_intake.onTrue(intakeSequence());

    // Button board column 2, row 2
    Trigger m_outtakeSpeaker = new Trigger(() -> m_buttonBoard.getRawButton(2));
    m_outtakeSpeaker.onTrue(outtakeSpeakerSequence(0));

    // Button board column 1, row 1
    Trigger m_cancelSubsystemCommands = new Trigger(() -> m_buttonBoard.getRawButton(3));
    m_cancelSubsystemCommands.onTrue(new InstantCommand(() -> cancelSubsystemCommands()));
  }

  public void cancelSubsystemCommands() {
    if (m_intakeSubsystem.getCurrentCommand() != null) {
      m_intakeSubsystem.getCurrentCommand().cancel();
    }
    if (m_outtakeSubsystem.getCurrentCommand() != null) {
      m_outtakeSubsystem.getCurrentCommand().cancel();
    }
  }

  public void setPose(double xPos, double yPos, double theta) {
    m_drivetrainSubsystem.setPose(xPos, yPos, theta);
    m_drivetrainSubsystem.alignTurningEncoders();
  }

  private void changePowerLimit(double delta) {
    if ((m_powerLimit <= 1.0 - Math.abs(delta) || delta <= 0) && (m_powerLimit >= Math.abs(delta) || delta >= 0)) {
      m_powerLimit += delta;
    }
  }

  private double getDPadInput(Joystick joystick) {
    if (joystick.getPOV() >= 315 || (joystick.getPOV() <= 45 && joystick.getPOV() >= 0)) {
      return 1.0;
    }
    if (joystick.getPOV() >= 135 && joystick.getPOV() <= 225) {
      return -1.0;
    }
    return 0;
  }

  private Command intakeSequence() {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new AutonIntakeCommand(m_intakeSubsystem, -200, -2.80, 2000),
        new SequentialCommandGroup(
          new WaitCommand(1.0),
          new PositionDriveCommand(m_drivetrainSubsystem, m_drivetrainSubsystem.getPosition().getX() + 0.25, m_drivetrainSubsystem.getPosition().getY(), m_drivetrainSubsystem.getAngle().getRadians(), 1000)
        )
      ),
      new AutonIntakeCommand(m_intakeSubsystem, 0, 0, 1000)
    );
  }

  private Command outtakeSpeakerSequence(double bumperToSpeakerEdgeDistance) {
    double outtakeAngle = -0.19989698802 * bumperToSpeakerEdgeDistance - 2.97675;
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        new AutonOuttakeCommand(m_outtakeSubsystem, OuttakeSubsystem.kOuttakeMaxRate, outtakeAngle, 1000),
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new AutonIntakeCommand(m_intakeSubsystem, 200, 500)
        )
      )
    );
  }
}