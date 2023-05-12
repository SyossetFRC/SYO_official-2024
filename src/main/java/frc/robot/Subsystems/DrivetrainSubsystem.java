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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Represents a swerve drive style drivetrain. */
public class DrivetrainSubsystem extends SubsystemBase {
  private static final double kTrackWidth = 0.51; // meters

  public static final double kMaxSpeed = (5676.0 / 60.0) * SwerveModule.kGearRatio * SwerveModule.kWheelRadius * 2 * Math.PI; // meters per second
  public static final double kMaxAngularSpeed = kMaxSpeed / Math.hypot(kTrackWidth / 2.0, kTrackWidth / 2.0); // radians per second

  private static final Translation2d m_frontLeftLocation = new Translation2d(kTrackWidth / 2.0, kTrackWidth / 2.0);
  private static final Translation2d m_frontRightLocation = new Translation2d(kTrackWidth / 2.0, -kTrackWidth / 2.0);
  private static final Translation2d m_backLeftLocation = new Translation2d(-kTrackWidth / 2.0, kTrackWidth / 2.0);
  private static final Translation2d m_backRightLocation = new Translation2d(-kTrackWidth / 2.0, -kTrackWidth / 2.0);

  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;

  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200);

  private final SwerveDriveKinematics m_kinematics;

  private final SwerveDriveOdometry m_odometry;

  private final GenericEntry m_frontLeftDriveSpeedEntry;
  private final GenericEntry m_frontLeftSteerAngleEntry;
  private final GenericEntry m_frontRightDriveSpeedEntry;
  private final GenericEntry m_frontRightSteerAngleEntry;
  private final GenericEntry m_backLeftDriveSpeedEntry;
  private final GenericEntry m_backLeftSteerAngleEntry;
  private final GenericEntry m_backRightDriveSpeedEntry;
  private final GenericEntry m_backRightSteerAngleEntry;
  private final GenericEntry m_odometryXEntry;
  private final GenericEntry m_odometryYEntry;
  private final GenericEntry m_odometryThetaEntry;

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

    m_odometry = new SwerveDriveOdometry(m_kinematics, m_navx.getRotation2d(), getModulePositions());

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    ShuffleboardLayout frontLeftLayout = tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 2).withPosition(0, 0);
    m_frontLeftDriveSpeedEntry = frontLeftLayout.add("Drive Speed", m_frontLeft.getState().speedMetersPerSecond).getEntry();
    m_frontLeftSteerAngleEntry = frontLeftLayout.add("Steer Angle", m_frontLeft.getState().angle.getDegrees()).getEntry();
    
    ShuffleboardLayout frontRightLayout = tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 2).withPosition(2, 0);
    m_frontRightDriveSpeedEntry = frontRightLayout.add("Drive Speed", m_frontRight.getState().speedMetersPerSecond).getEntry();
    m_frontRightSteerAngleEntry = frontRightLayout.add("Steer Angle", m_frontRight.getState().angle.getDegrees()).getEntry();

    ShuffleboardLayout backLeftLayout = tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 2).withPosition(4, 0);
    m_backLeftDriveSpeedEntry = backLeftLayout.add("Drive Speed", m_backLeft.getState().speedMetersPerSecond).getEntry();
    m_backLeftSteerAngleEntry = backLeftLayout.add("Steer Angle", m_backLeft.getState().angle.getDegrees()).getEntry();

    ShuffleboardLayout backRightLayout = tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 2).withPosition(6, 0);
    m_backRightDriveSpeedEntry = backRightLayout.add("Drive Speed", m_backRight.getState().speedMetersPerSecond).getEntry();
    m_backRightSteerAngleEntry = backRightLayout.add("Steer Angle", m_backRight.getState().angle.getDegrees()).getEntry();

    ShuffleboardLayout odometryLayout = tab.getLayout("Odometry", BuiltInLayouts.kList).withSize(2, 3).withPosition(0, 2);
    m_odometryXEntry = odometryLayout.add("X Position", getPosition().getX()).getEntry();
    m_odometryYEntry = odometryLayout.add("Y Position", getPosition().getY()).getEntry();
    m_odometryThetaEntry = odometryLayout.add("Angle", getAngle().getDegrees()).getEntry();
  }

  /**
   * Drives the robot using joystick info.
   *
   * @param xSpeed The speed of the robot in the x direction (m/s).
   * @param ySpeed The speed of the robot in the y direction (m/s).
   * @param rot The angular rate of the robot (rad/s).
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_rot = rot;
    m_fieldRelative = fieldRelative;
  }

  /** Returns the current odometric position of the robot.
   * 
   * @return The current odometric position of the robot.
   */
  public Translation2d getPosition() {
    return m_odometry.getPoseMeters().getTranslation();
  }

  /** Returns the current odometric angle of the robot. 
   * 
   * @return The current odometric angle of the robot.
   */
  public Rotation2d getAngle() {
    return m_odometry.getPoseMeters().getRotation();
  }

  /**
   * Sets the odometric position and angle of the robot.
   *
   * @param xPos The position of the robot in the x direction (m).
   * @param yPos The position of the robot in the y direction (m).
   * @param theta The angle of the robot (rad).
   */
  public void setPose(double xPos, double yPos, double theta) {
    m_odometry.resetPosition(m_navx.getRotation2d(), getModulePositions(), new Pose2d(xPos, yPos, new Rotation2d(theta)));
  }

  /** Changes the drive motor idle modes.
   * 
   * @param idleMode The idle mode to set the drive motors to: "brake" -> kBrake and "coast" -> kCoast.
   */
  public void setIdleMode(String idleMode) {
    m_frontLeft.setIdleMode(idleMode);
    m_frontRight.setIdleMode(idleMode);
    m_backLeft.setIdleMode(idleMode);
    m_backRight.setIdleMode(idleMode);
  }

  /** Returns initial positions of the swerve modules as a SwerveModulePosition[].
   * 
   * @return The initial positions of the swerve modules as a SwerveModulePosition[].
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getDrivePosition(),
        m_frontRight.getDrivePosition(),
        m_backLeft.getDrivePosition(),
        m_backRight.getDrivePosition()
    };
  }

  /** Displays the periodically updated robot poses on the Shuffleboard */
  public void updateShuffleboard() {
    m_frontLeftDriveSpeedEntry.setDouble(m_frontLeft.getState().speedMetersPerSecond);
    m_frontLeftSteerAngleEntry.setDouble(m_frontLeft.getState().angle.getDegrees());

    m_frontRightDriveSpeedEntry.setDouble(m_frontRight.getState().speedMetersPerSecond);
    m_frontRightSteerAngleEntry.setDouble(m_frontRight.getState().angle.getDegrees());

    m_backLeftDriveSpeedEntry.setDouble(m_backLeft.getState().speedMetersPerSecond);
    m_backLeftSteerAngleEntry.setDouble(m_backLeft.getState().angle.getDegrees());

    m_backRightDriveSpeedEntry.setDouble(m_backRight.getState().speedMetersPerSecond);
    m_backRightSteerAngleEntry.setDouble(m_backRight.getState().angle.getDegrees());

    m_odometryXEntry.setDouble(getPosition().getX());
    m_odometryYEntry.setDouble(getPosition().getY());
    m_odometryThetaEntry.setDouble(getAngle().getDegrees());
  }

  @Override
  public void periodic() {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            m_fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(m_xSpeed, m_ySpeed, m_rot, getAngle())
                : new ChassisSpeeds(m_xSpeed, m_ySpeed, m_rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    m_odometry.update(
        m_navx.getRotation2d(),
        getModulePositions()
    );

    updateShuffleboard();
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
    private final PIDController m_turningPIDController = new PIDController(3.0, 0, 0.1);
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.18868, 0.12825);

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
     *
     * @param driveMotorChannel The CAN output for the drive motor.
     * @param turningMotorChannel The CAN output for the turning motor.
     * @param turningEncoderChannel The CAN input for the turning encoder.
     * @param moduleOffset The angle offset for the turning encoder (rad).
     */
    private SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel, double moduleOffset) {
      m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
      m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

      m_driveMotor.setIdleMode(IdleMode.kBrake);
      m_turningMotor.setIdleMode(IdleMode.kBrake);

      m_driveEncoder = m_driveMotor.getEncoder();
      m_turningEncoder = new CANCoder(turningEncoderChannel);

      m_driveEncoder.setPositionConversionFactor(kGearRatio * kWheelRadius * 2 * Math.PI); // meters
      m_driveEncoder.setVelocityConversionFactor(kGearRatio * kWheelRadius * 2 * Math.PI / 60.0); // meters per second

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
      return new SwerveModulePosition(m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getAbsolutePosition() - m_moduleOffset));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState The desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
      // Optimizes the reference state to avoid spinning further than 90 degrees.
      SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getAbsolutePosition() - m_moduleOffset));

      // Calculates the turning motor output from the turning PID controller.
      final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getAbsolutePosition() - m_moduleOffset, state.angle.getRadians());
      m_turningMotor.setVoltage(turnOutput);

      // Updates velocity based on turn error.
      state.speedMetersPerSecond *= Math.cos(m_turningPIDController.getPositionError());

      // Calculates the drive output from the drive PID controller and feedforward controller.
      final double speedRadiansPerSecond = state.speedMetersPerSecond / kWheelRadius;
      final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity() / kWheelRadius, speedRadiansPerSecond);
      final double driveFeedForward = m_driveFeedforward.calculate(speedRadiansPerSecond);
      m_driveMotor.setVoltage(driveOutput + driveFeedForward);
    }

    /** Changes the drive motor idle mode.
     * 
     * @param idleMode The idle mode to set the drive motor to: "brake" -> kBrake and "coast" -> kCoast.
     */
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