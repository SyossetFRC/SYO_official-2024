package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.AutonIntakeCommand;
import frc.robot.Commands.LimelightOuttakeCommand;
import frc.robot.Commands.LimelightRotateCommand;
import frc.robot.Commands.AutonOuttakeCommand;
import frc.robot.Commands.BrakeCommand;
import frc.robot.Commands.DefaultClimberCommand;
import frc.robot.Commands.DefaultDriveCommand;
import frc.robot.Commands.DefaultIntakeCommand;
import frc.robot.Commands.DefaultOuttakeCommand;
import frc.robot.Commands.PositionDriveCommand;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;
import frc.robot.Subsystems.OuttakeSubsystem;

/** Represents the entire robot. */
public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final OuttakeSubsystem m_outtakeSubsystem = new OuttakeSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();

  private final UsbCamera m_camera;

  private final Joystick m_driveController = new Joystick(0);
  private final Joystick m_operatorController = new Joystick(1);
  private final Joystick m_buttonBoard = new Joystick(2);
  private double m_powerLimit = 0.8;

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
        () -> getDPadInput(m_operatorController) * IntakeSubsystem.kIntakeMaxRate * 0.25, 
        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(5), 0.05) * IntakeSubsystem.kRotateMaxAngularSpeed * 0.15
    ));

    m_outtakeSubsystem.setDefaultCommand(new DefaultOuttakeCommand(
        m_outtakeSubsystem, 
        () -> MathUtil.applyDeadband(m_operatorController.getRawAxis(3), 0.05) * OuttakeSubsystem.kOuttakeMaxRate * 0.69,
        () -> -MathUtil.applyDeadband(m_operatorController.getRawAxis(1), 0.05)
    ));

    m_climberSubsystem.setDefaultCommand(new DefaultClimberCommand(
        m_climberSubsystem,
        () -> getDPadInput(m_buttonBoard) * MathUtil.applyDeadband(m_buttonBoard.getRawAxis(3), 0.05) * 0.50,
        () -> getDPadInput(m_buttonBoard) * MathUtil.applyDeadband(m_buttonBoard.getRawAxis(2), 0.05) * 0.50
    ));

    m_camera = CameraServer.startAutomaticCapture();
    m_camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    m_camera.setResolution(320, 240);
    Shuffleboard.getTab("Match").add(CameraServer.getServer().getSource()).withWidget(BuiltInWidgets.kCameraStream).withSize(7, 5).withPosition(4, 0);

    configureButtons();
  }

  /**
   * Command sequence to run in autonomous. The origin is the center of the front subwoofer edge.
   * 
   * @param startX Starting X Position (m).
   * @param startY Starting Y Position (m).
   * @param startTheta Starting angle (rad).
   * @param autonomousNotes ArrayList containing notes to be scored in order.
   * @return Command to run autonomously.
   */
  public Command autonomousCommands(double startX, double startY, double startTheta, ArrayList<SpikeMarkNote> autonomousNotes) {
    setPose(startX, startY, startTheta);
    m_drivetrainSubsystem.alignTurningEncoders();
    m_intakeSubsystem.reset();

    SequentialCommandGroup autonomousSequence = new SequentialCommandGroup(
      new ParallelCommandGroup(
        new AutonOuttakeCommand(m_outtakeSubsystem, OuttakeSubsystem.kOuttakeMaxRate * 0.69, -1.98, 1500),
        new SequentialCommandGroup(
          new WaitCommand(1.0),
          new AutonIntakeCommand(m_intakeSubsystem, 400, 0, 500)
        )
      )
    );
    for (SpikeMarkNote note : autonomousNotes) {
      switch(note) {
        case LEFT:
          autonomousSequence.addCommands(leftNoteSequence());
          break;
        case MIDDLE:
          autonomousSequence.addCommands(middleNoteSequence());
          break;
        case RIGHT:
          autonomousSequence.addCommands(rightNoteSequence());
          break;
      }
    }
    return autonomousSequence;
  }

  /**
   * Configures all controller buttons.
   */
  private void configureButtons() {
    // Driver button A
    Trigger m_resetPose = new Trigger(() -> m_driveController.getRawButton(1));
    m_resetPose.onTrue(new InstantCommand(() -> setPose(0, 0, 0)));

    // Operator button A & Button board column 3, row 1
    Trigger m_resetSubsystems = new Trigger(() -> (m_operatorController.getRawButton(1) || m_buttonBoard.getRawButton(6)));
    m_resetSubsystems.onTrue(new InstantCommand(() -> m_intakeSubsystem.reset()));

    // Driver button X
    Trigger m_brake = new Trigger(() -> m_driveController.getRawButton(3));
    m_brake.onTrue(new BrakeCommand(m_drivetrainSubsystem));
    m_brake.onFalse(new InstantCommand(() -> m_drivetrainSubsystem.getCurrentCommand().cancel()));

    // Driver D-pad up
    Trigger m_incrementPowerLimit = new Trigger(() -> getDPadInput(m_driveController) == 1.0);
    m_incrementPowerLimit.onTrue(new InstantCommand(() -> changePowerLimit(0.1)));

    // Driver D-pad down
    Trigger m_decrementPowerLimit = new Trigger(() -> getDPadInput(m_driveController) == -1.0);
    m_decrementPowerLimit.onTrue(new InstantCommand(() -> changePowerLimit(-0.1)));

    // Button board column 1, row 2
    Trigger m_intake = new Trigger(() -> m_buttonBoard.getRawButton(1));
    m_intake.whileTrue(new AutonIntakeCommand(m_intakeSubsystem, -400, -2.80, 150000));
    m_intake.onFalse(new AutonIntakeCommand(m_intakeSubsystem, 0, 0, 1000));

    // Button board column 1, row 1
    Trigger m_outtakeSpeaker = new Trigger(() -> m_buttonBoard.getRawButton(3));
    m_outtakeSpeaker.onTrue(new ParallelCommandGroup(
      new AutonOuttakeCommand(m_outtakeSubsystem, OuttakeSubsystem.kOuttakeMaxRate * 0.69, -1.98, 1500),
      new SequentialCommandGroup(
        new WaitCommand(1),
        new AutonIntakeCommand(m_intakeSubsystem, 400, 0, 500)
      )
    ));

    // Button board column 2, row 2
    Trigger m_outtakeLimelightSpeaker = new Trigger(() -> m_buttonBoard.getRawButton(2));
    m_outtakeLimelightSpeaker.onTrue(new ParallelCommandGroup(
      new ParallelCommandGroup(
        new LimelightOuttakeCommand(m_outtakeSubsystem, m_limelightSubsystem, OuttakeSubsystem.kOuttakeMaxRate * 0.69, 1500),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new AutonIntakeCommand(m_intakeSubsystem, 400, 0, 500)
        )
      ),
      new LimelightRotateCommand(m_drivetrainSubsystem, m_limelightSubsystem, 1500)
    ));

    // Button board column 4, row 1
    Trigger m_outtakeAmp = new Trigger(() -> m_buttonBoard.getRawButton(5));
    m_outtakeAmp.onTrue(new SequentialCommandGroup(
      new AutonIntakeCommand(m_intakeSubsystem, -200, 0, 250),
      new WaitCommand(1.0),
      new AutonIntakeCommand(m_intakeSubsystem, 0, -2.80, 225),
      new AutonIntakeCommand(m_intakeSubsystem, IntakeSubsystem.kIntakeMaxRate * 0.68, -0.93, 500),
      new AutonIntakeCommand(m_intakeSubsystem, 0, 0, 275)
    ));

    // Button board column 2, row 1
    Trigger m_cancelSubsystemCommands = new Trigger(() -> m_buttonBoard.getRawButton(4));
    m_cancelSubsystemCommands.onTrue(new InstantCommand(() -> cancelSubsystemCommands()));
  }

  /**
   * Cancels all subsystem commands and presets.
   */
  private void cancelSubsystemCommands() {
    if (m_intakeSubsystem.getCurrentCommand() != null) {
      m_intakeSubsystem.getCurrentCommand().cancel();
    }
    if (m_outtakeSubsystem.getCurrentCommand() != null) {
      m_outtakeSubsystem.getCurrentCommand().cancel();
    }
  }

  /**
   * Sets robot odometric position.
   * 
   * @param xPos X Position (m).
   * @param yPos Y Position (m).
   * @param theta Angle (rad).
   */
  private void setPose(double xPos, double yPos, double theta) {
    m_drivetrainSubsystem.setPose(xPos, yPos, theta);
    m_drivetrainSubsystem.alignTurningEncoders();
  }

  /**
   * Changes the drive controller power limit [0, 1].
   * 
   * @param delta Power Limit Change.
   */
  private void changePowerLimit(double delta) {
    if ((m_powerLimit <= 1.0 - Math.abs(delta) || delta <= 0) && (m_powerLimit >= Math.abs(delta) || delta >= 0)) {
      m_powerLimit += delta;
    }
  }

  /**
   * Converts D-Pad input into either +1 or -1 output.
   * 
   * @param joystick Joystick to access D-Pad.
   * @return D-Pad up returns +1 and D-Pad down returns -1.
   */
  private double getDPadInput(Joystick joystick) {
    if (joystick.getPOV() >= 315 || (joystick.getPOV() <= 45 && joystick.getPOV() >= 0)) {
      return 1.0;
    }
    if (joystick.getPOV() >= 135 && joystick.getPOV() <= 225) {
      return -1.0;
    }
    return 0;
  }
 
  // private Command leftOutNoteSequence(){
  //   return new SequentialCommandGroup(
  //     new PositionDriveCommand(m_drivetrainSubsystem, 7.79, 0.753, Math.PI/2, 6000),
  //     new ParallelCommandGroup(
  //       new AutonIntakeCommand(m_intakeSubsystem, -400, -2.80, 2000),
  //       new SequentialCommandGroup(
  //         new WaitCommand(1),
  //         new PositionDriveCommand(m_drivetrainSubsystem, 8.29, 0.753, 0, 1000)
  //       )
  //     ),
  //     new ParallelCommandGroup(
  //       new AutonIntakeCommand(m_intakeSubsystem, 0, 0, 1000),
  //       new PositionDriveCommand(m_drivetrainSubsystem, 1.10, 1.00, 0.460, 1000)
  //     ),
  //     new ParallelCommandGroup(
  //       new ParallelCommandGroup(
  //         new LimelightOuttakeCommand(m_outtakeSubsystem, m_limelightSubsystem, OuttakeSubsystem.kOuttakeMaxRate * 0.69, 1500),
  //         new SequentialCommandGroup(
  //           new WaitCommand(1),
  //           new AutonIntakeCommand(m_intakeSubsystem, 400, 0, 500)
  //         )
  //       ),
  //       new LimelightRotateCommand(m_drivetrainSubsystem, m_limelightSubsystem, 1500)
  //     )
  //   );
  // }

  private Command leftNoteSequence() {
    return new SequentialCommandGroup(
      new PositionDriveCommand(m_drivetrainSubsystem, 1.10, 1.50, 0, 1.5, Math.PI / 2,4000),
      new ParallelCommandGroup(
        new AutonIntakeCommand(m_intakeSubsystem, -400, -2.80, 2000),
        new SequentialCommandGroup(
          new WaitCommand(1.0),
          new PositionDriveCommand(m_drivetrainSubsystem, 1.60, 1.50, 0, 1000)
        )
      ),
      new ParallelCommandGroup(
        new AutonIntakeCommand(m_intakeSubsystem, 0, 0, 1000),
        new PositionDriveCommand(m_drivetrainSubsystem, 1.10, 1.00, 0.460, 1000)
      ),
      new ParallelCommandGroup(
        new ParallelCommandGroup(
          new LimelightOuttakeCommand(m_outtakeSubsystem, m_limelightSubsystem, OuttakeSubsystem.kOuttakeMaxRate * 0.69, 1500),
          new SequentialCommandGroup(
            new WaitCommand(1),
            new AutonIntakeCommand(m_intakeSubsystem, 400, 0, 500)
          )
        ),
        new LimelightRotateCommand(m_drivetrainSubsystem, m_limelightSubsystem, 1500)
      )
    );
  }

  /**
   * Command to intake and shoot the note on the middle spike mark of either alliance, from the POV of the drivers.
   * 
   * @return Command to intake and shoot the note on the middle spike mark of either alliance, from the POV of the drivers.
   */
  private Command middleNoteSequence() {
    return new SequentialCommandGroup(
      new PositionDriveCommand(m_drivetrainSubsystem, 1.10, 0, 0, 1.5, Math.PI / 2,4000),
      new ParallelCommandGroup(
        new AutonIntakeCommand(m_intakeSubsystem, -400, -2.80, 2000),
        new SequentialCommandGroup(
          new WaitCommand(1.0),
          new PositionDriveCommand(m_drivetrainSubsystem, 1.60, 0, 0, 1000)
        )
      ),
      new ParallelCommandGroup(
        new AutonIntakeCommand(m_intakeSubsystem, 0, 0, 1000),
        new PositionDriveCommand(m_drivetrainSubsystem, 1.10, 0, 0, 1000)
      ),
      new ParallelCommandGroup(
        new ParallelCommandGroup(
          new LimelightOuttakeCommand(m_outtakeSubsystem, m_limelightSubsystem, OuttakeSubsystem.kOuttakeMaxRate * 0.69, 1500),
          new SequentialCommandGroup(
            new WaitCommand(1),
            new AutonIntakeCommand(m_intakeSubsystem, 400, 0, 500)
          )
        ),
        new LimelightRotateCommand(m_drivetrainSubsystem, m_limelightSubsystem, 1500)
      )
    );
  }

  /**
   * Command to intake and shoot the note on the right-most spike mark of either alliance, from the POV of the drivers.
   * 
   * @return Command to intake and shoot the note on the right-most spike mark of either alliance, from the POV of the drivers.
   */
  private Command rightNoteSequence() {
    return new SequentialCommandGroup(
      new PositionDriveCommand(m_drivetrainSubsystem, 1.10, -1.50, 0, 1.5, Math.PI / 2, 4000),
      new ParallelCommandGroup(
        new AutonIntakeCommand(m_intakeSubsystem, -400, -2.80, 2000),
        new SequentialCommandGroup(
          new WaitCommand(1.0),
          new PositionDriveCommand(m_drivetrainSubsystem, 1.60, -1.50, 0, 1000)
        )
      ),
      new ParallelCommandGroup(
        new AutonIntakeCommand(m_intakeSubsystem, 0, 0, 1000),
        new PositionDriveCommand(m_drivetrainSubsystem, 1.10, -1.00, -0.460, 1000)
      ),
      new ParallelCommandGroup(
        new ParallelCommandGroup(
          new LimelightOuttakeCommand(m_outtakeSubsystem, m_limelightSubsystem, OuttakeSubsystem.kOuttakeMaxRate * 0.69, 1500),
          new SequentialCommandGroup(
            new WaitCommand(1),
            new AutonIntakeCommand(m_intakeSubsystem, 400, 0, 500)
          )
        ),
        new LimelightRotateCommand(m_drivetrainSubsystem, m_limelightSubsystem, 1500)
      )
    );
  }
}

/* Autonomous spike mark note options */
enum SpikeMarkNote {
  LEFT,
  MIDDLE,
  RIGHT
}