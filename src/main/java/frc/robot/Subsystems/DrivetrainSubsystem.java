// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Represents a swerve drive style drivetrain. */
public class DrivetrainSubsystem extends SubsystemBase {
  private static final double kTrackWidth = 0.51;

  public static final double kMaxSpeed = (5676.0 / 60.0) * SwerveModule.kGearRatio * SwerveModule.kWheelRadius * 2 * Math.PI; // meters per second
  public static final double kMaxAngularSpeed = kMaxSpeed / Math.hypot(kTrackWidth / 2.0, kTrackWidth / 2.0); // radians per second

  private final Translation2d m_frontLeftLocation = new Translation2d(kTrackWidth / 2.0, kTrackWidth / 2.0);
  private final Translation2d m_frontRightLocation = new Translation2d(kTrackWidth / 2.0, -kTrackWidth / 2.0);
  private final Translation2d m_backLeftLocation = new Translation2d(-kTrackWidth / 2.0, kTrackWidth / 2.0);
  private final Translation2d m_backRightLocation = new Translation2d(-kTrackWidth / 2.0, -kTrackWidth / 2.0);

  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);

  private final SwerveDriveKinematics m_kinematics;

  private final SwerveDriveOdometry m_odometry;

  private double m_xSpeed;
  private double m_ySpeed;
  private double m_rot;
  private boolean m_fieldRelative;

  public DrivetrainSubsystem() {
    m_frontLeft = new SwerveModule(Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR, Constants.FRONT_LEFT_MODULE_STEER_MOTOR, Constants.FRONT_LEFT_MODULE_STEER_ENCODER, Constants.FRONT_LEFT_MODULE_STEER_OFFSET);
    m_frontRight = new SwerveModule(Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR, Constants.FRONT_RIGHT_MODULE_STEER_MOTOR, Constants.FRONT_RIGHT_MODULE_STEER_ENCODER, Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);
    m_backLeft = new SwerveModule(Constants.BACK_LEFT_MODULE_DRIVE_MOTOR, Constants.BACK_LEFT_MODULE_STEER_MOTOR, Constants.BACK_LEFT_MODULE_STEER_ENCODER, Constants.BACK_LEFT_MODULE_STEER_OFFSET);
    m_backRight = new SwerveModule(Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR, Constants.BACK_RIGHT_MODULE_STEER_MOTOR, Constants.BACK_RIGHT_MODULE_STEER_ENCODER, Constants.BACK_RIGHT_MODULE_STEER_OFFSET);

    m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    m_odometry = new SwerveDriveOdometry(m_kinematics, m_navx.getRotation2d(), getModulePositions(), new Pose2d(0, 0, new Rotation2d(0)));

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    ShuffleboardLayout frontLeftLayout = tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 2).withPosition(0, 0);
    frontLeftLayout.add("Drive Speed", m_frontLeft.getState().speedMetersPerSecond);
    frontLeftLayout.add("Steer Angle", m_frontLeft.getState().angle.getDegrees());
    
    ShuffleboardLayout frontRightLayout = tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 2).withPosition(2, 0);
    frontRightLayout.add("Drive Speed", m_frontRight.getState().speedMetersPerSecond);
    frontRightLayout.add("Steer Angle", m_frontRight.getState().angle.getDegrees());

    ShuffleboardLayout backLeftLayout = tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 2).withPosition(4, 0);
    backLeftLayout.add("Drive Speed", m_backLeft.getState().speedMetersPerSecond);
    backLeftLayout.add("Steer Angle", m_backLeft.getState().angle.getDegrees());

    ShuffleboardLayout backRightLayout = tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 2).withPosition(6, 0);
    backRightLayout.add("Drive Speed", m_backRight.getState().speedMetersPerSecond);
    backRightLayout.add("Steer Angle", m_backRight.getState().angle.getDegrees());

    ShuffleboardLayout odometryLayout = tab.getLayout("Odometry", BuiltInLayouts.kList).withSize(2, 2).withPosition(0, 2);
    odometryLayout.add("X Position", m_odometry.getPoseMeters().getX());
    odometryLayout.add("Y Position", m_odometry.getPoseMeters().getY());

    tab.add("Gyroscope Angle", m_navx.getRotation2d().getDegrees()).withPosition(2, 2);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_rot = rot;
    m_fieldRelative = fieldRelative;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_navx.getRotation2d(),
        getModulePositions()
    );
  }

  /** Returns the current odometric position of the robot. */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /** Resets the gyroscope angle to re-adjust drive POV. */
  public void resetGyroscope() {
    m_navx.zeroYaw();
  }

  /** Returns the gyroscope angle. */
  public Rotation2d getAngle() {
    return m_navx.getRotation2d();
  }

  /** Changes the drive motor idle modes. */
  public void setIdleMode(String idleMode) {
    m_frontLeft.setIdleMode(idleMode);
    m_frontRight.setIdleMode(idleMode);
    m_backLeft.setIdleMode(idleMode);
    m_backRight.setIdleMode(idleMode);
  }

  /** Returns initial positions of the swerve modules as SwerveModulePosition[] */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getDrivePosition(),
        m_frontRight.getDrivePosition(),
        m_backLeft.getDrivePosition(),
        m_backRight.getDrivePosition()
    };
  }

  @Override
  public void periodic() {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            m_fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(m_xSpeed, m_ySpeed, m_rot, m_navx.getRotation2d())
                : new ChassisSpeeds(m_xSpeed, m_ySpeed, m_rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    updateOdometry();
  }

  private class SwerveModule {
    private static final double kWheelRadius = 0.050165; // meters
    private static final double kGearRatio = 0.12280701754;

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final RelativeEncoder m_driveEncoder;
    private final CANCoder m_turningEncoder;
    private final double m_moduleOffset;

    private final PIDController m_drivePIDController = new PIDController(0.1, 0, 0);
    private final PIDController m_turningPIDController = new PIDController(10.0, 0, 0);
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.18868, 0.12825);

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
     *
     * @param driveMotorChannel CAN output for the drive motor
     * @param turningMotorChannel CAN output for the turning motor
     * @param turningEncoderChannel CAN input for the turning encoder
     * @param moduleOffset Angle offset for the turning encoder (rad)
     */
    private SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel, double moduleOffset) {
      m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
      m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

      m_driveMotor.setIdleMode(IdleMode.kBrake);
      m_turningMotor.setIdleMode(IdleMode.kBrake);

      m_driveEncoder = m_driveMotor.getEncoder();
      m_turningEncoder = new CANCoder(turningEncoderChannel);

      m_driveEncoder.setPositionConversionFactor(kGearRatio * SwerveModule.kWheelRadius * 2 * Math.PI); // meters
      m_driveEncoder.setVelocityConversionFactor(kGearRatio * SwerveModule.kWheelRadius * 2 * Math.PI / 60.0); // meters per second

      m_turningEncoder.configFeedbackCoefficient(0.0015339807878818, "rad", SensorTimeBase.PerSecond);
      m_moduleOffset = moduleOffset;

      m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
      return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getAbsolutePosition() - m_moduleOffset));
    }

    /**
     * Returns the current distance of the drive encoder in meters as a SwerveModulePosition.
     *
     * @return The current distance of the drive encoder in meters as a SwerveModulePosition.
     */
    public SwerveModulePosition getDrivePosition() {
      return new SwerveModulePosition(m_driveEncoder.getPosition(), getState().angle);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
      // Optimize the reference state to avoid spinning further than 90 degrees
      SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getAbsolutePosition() - m_moduleOffset));

      // Calculate the drive output from the drive PID controller and feedforward controller.
      double speedRadiansPerSecond = state.speedMetersPerSecond / kWheelRadius;
      final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity() / kWheelRadius, speedRadiansPerSecond);
      final double driveFeedForward = m_driveFeedforward.calculate(speedRadiansPerSecond);

      // Calculate the turning motor output from the turning PID controller.
      final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getAbsolutePosition() - m_moduleOffset, state.angle.getRadians());

      m_driveMotor.setVoltage(driveOutput + driveFeedForward);
      m_turningMotor.setVoltage(turnOutput);
    }

    public void setIdleMode(String idleMode) {
      if (idleMode.equals("brake")) {
        m_driveMotor.setIdleMode(IdleMode.kBrake);
      }
      if (idleMode.equals("coast")) {
        m_driveMotor.setIdleMode(IdleMode.kCoast);
      }
    }
  }
}