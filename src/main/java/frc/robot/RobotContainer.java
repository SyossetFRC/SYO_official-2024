package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Commands.BrakeCommand;
import frc.robot.Commands.DefaultDriveCommand;
import frc.robot.Commands.IdleDriveCommand;
import frc.robot.Commands.PositionDriveCommand;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class RobotContainer {
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
    
    private final Joystick m_driveController = new Joystick(0);
    private double m_rotatePower;

    public RobotContainer() {
      m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
          m_drivetrainSubsystem,
          () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(1), 0.05) * (-m_driveController.getRawAxis(3) + 1) * DrivetrainSubsystem.kMaxSpeed,
          () -> -MathUtil.applyDeadband(m_driveController.getRawAxis(0), 0.05) * (-m_driveController.getRawAxis(3) + 1) * DrivetrainSubsystem.kMaxSpeed,
          () -> m_rotatePower * (-m_driveController.getRawAxis(3) + 1) * DrivetrainSubsystem.kMaxAngularSpeed
      ));

      configureButtons();
    }

    public SequentialCommandGroup autonomousCommands() {
      return new SequentialCommandGroup(
          new PositionDriveCommand(m_drivetrainSubsystem, 3.5, 0.2, Math.PI / 2, 2.5, Math.PI / 6),
          new IdleDriveCommand(m_drivetrainSubsystem, 0.2),
          new PositionDriveCommand(m_drivetrainSubsystem, 4, 0.5, Math.PI / 2, 0.5, Math.PI / 4)
      );
    }

    private void configureButtons() {
      Button m_resetPose = new Button(() -> m_driveController.getRawButton(6));
      m_resetPose.whenPressed(() -> setPose(0, 0, 0));

      Button m_brake = new Button(() -> m_driveController.getRawButton(10));
      m_brake.whenPressed(new BrakeCommand(m_drivetrainSubsystem));
      m_brake.whenReleased(() -> m_drivetrainSubsystem.getCurrentCommand().cancel());

      Button m_rotateLeft = new Button(() -> m_driveController.getRawButton(11));
      m_rotateLeft.whenPressed(() -> setRotatePower("left"));
      m_rotateLeft.whenReleased(() -> setRotatePower("none"));

      Button m_rotateRight = new Button(() -> m_driveController.getRawButton(12));
      m_rotateRight.whenPressed(() -> setRotatePower("right"));
      m_rotateRight.whenReleased(() -> setRotatePower("none"));
    }

    public void setPose(double xPos, double yPos, double theta) {
      m_drivetrainSubsystem.setPose(xPos, yPos, theta);
    }

    public void setIdleMode(String idleMode) {
      m_drivetrainSubsystem.setIdleMode(idleMode);
    }

    private void setRotatePower(String state) {
      if (state.equals("left")) {
        m_rotatePower = 0.25;
      }
      else if (state.equals("right")) {
        m_rotatePower = -0.25;
      }
      else {
        m_rotatePower = 0;
      }
    }
}
