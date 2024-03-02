package frc.robot.Commands;

import java.util.Map;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.LimelightSubsystem;

public class LimelightAlignmentCommand extends Command {
    private final LimelightSubsystem m_limelightSubsystem;
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final double kRotSpeedFactor = 15.0; 
    private final double rotationThreshold = 2.0;

    private double m_xVel;  // Velocity of the robot in the x-axis
    private double m_yVel;  // Velocity of the robot in the y-axis
    private double m_rotVel;    // Velocity of the robot's rotation

    private String m_trackingMode;  // Mode of tracking (translational or rotational)
    private double m_distanceToTag = 3;  // Distance from the robot to the player

    GenericEntry xVelEntry;
    GenericEntry yVelEntry;
    GenericEntry rotVelEntry;
    GenericEntry trackingModeEntry;
    GenericEntry powerLimit;

    GenericEntry kPEntry;
    GenericEntry kIEntry;
    GenericEntry kDEntry;

    DoubleSupplier m_xVelocity;
    DoubleSupplier m_yVelocity;

    private final long m_maxTime;
    private long m_recordedTime;
    private boolean m_isTimeRecorded;

    public LimelightAlignmentCommand(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem,
            String trackingMode, DoubleSupplier xVel, DoubleSupplier yVel, long maxTime) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_limelightSubsystem = limelightSubsystem;
        m_trackingMode = trackingMode;

        m_xVelocity = xVel;
        m_yVelocity = yVel;

        ShuffleboardLayout tagTrackingLayout = Shuffleboard.getTab("Limelight")
                .getLayout("Tag Tracking Data", BuiltInLayouts.kList).withSize(2, 2);

        xVelEntry = tagTrackingLayout.add("X Velocity", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
        yVelEntry = tagTrackingLayout.add("Y Velocity", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
        rotVelEntry = tagTrackingLayout.add("Rotational Velocity", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
        // trackingModeEntry = tagTrackingLayout.add("Tracking Mode",
        // "Translational").withWidget(BuiltInWidgets.kTextView)
        // .getEntry();
        kPEntry = Shuffleboard.getTab("Limelight").add("kP Entry", 0.045).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1)).getEntry();

        kIEntry = Shuffleboard.getTab("Limelight").add("kI Entry", 0).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1)).getEntry();
        
        kDEntry = Shuffleboard.getTab("Limelight").add("kD Entry", 0).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1)).getEntry();

        powerLimit = Shuffleboard.getTab("Limelight").add("Power Limit", 1).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1)).getEntry();

        m_maxTime = maxTime;

        m_isTimeRecorded = false;

        addRequirements(m_drivetrainSubsystem);
    }

    public LimelightAlignmentCommand(DrivetrainSubsystem drivetrainSubsystem, LimelightSubsystem limelightSubsystem, long maxTime) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_limelightSubsystem = limelightSubsystem;

        ShuffleboardLayout tagTrackingLayout = Shuffleboard.getTab("Limelight")
                .getLayout("Tag Tracking Data", BuiltInLayouts.kList).withSize(2, 2);

        xVelEntry = tagTrackingLayout.add("X Velocity", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
        yVelEntry = tagTrackingLayout.add("Y Velocity", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
        rotVelEntry = tagTrackingLayout.add("Rotational Velocity", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
        trackingModeEntry = tagTrackingLayout.add("Tracking Mode", "Translational").withWidget(BuiltInWidgets.kTextView)
                .getEntry();

        powerLimit = Shuffleboard.getTab("Limelight").add("Power Limit", 1).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1)).getEntry();

        m_maxTime = maxTime;

        m_isTimeRecorded = false;

        addRequirements(m_drivetrainSubsystem);
    }


    @Override
    public void execute() {
        if (!m_isTimeRecorded) {
                m_recordedTime = System.currentTimeMillis();
                m_isTimeRecorded = true;
        }

        // m_xVel = MathUtil.applyDeadband(m_xPID.calculate(m_limelightSubsystem.getXTargetAngle()), 0.05)
        //         * powerLimit.getDouble(1); // Calculate the velocity of the robot in the x-axis
        // m_yVel = MathUtil.applyDeadband(-m_yPID.calculate(m_limelightSubsystem.getDistance("Area") - m_distanceToTag),
        //         0.05) * 2.5 * powerLimit.getDouble(1); // Calculate the velocity of the robot in the y-axis

        m_xVel = m_xVelocity.getAsDouble();
        m_yVel = m_yVelocity.getAsDouble();

        double targetAngle = m_limelightSubsystem.getXTargetAngle();
        if (Math.abs(targetAngle) > rotationThreshold) {
                m_rotVel = -m_limelightSubsystem.getXTargetAngle() / 180.0 * kRotSpeedFactor;
        } else {
                m_rotVel = 0;
        }

        // m_trackingMode = m_limelightSubsystem.getTrackingMode();
        // m_trackingMode = m_limelightSubsystem.getTrackingMode();
        m_distanceToTag = m_limelightSubsystem.getDistanceToTag();

        m_drivetrainSubsystem.drive(
                m_trackingMode.equals("rotational") ? m_xVel : 0,
                m_trackingMode.equals("rotational") ? m_yVel : 0,
                m_trackingMode.equals("rotational") ? m_rotVel : 0,
                m_trackingMode.equals("translational")); // Drives the robot based on the tracking mode selected

        xVelEntry.setDouble(m_xVel); // Shuffleboard data
        yVelEntry.setDouble(m_yVel);
        rotVelEntry.setDouble(m_rotVel);
        // trackingModeEntry.setString(m_trackingMode);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(m_limelightSubsystem.getDistance("Area") - m_distanceToTag) <= 0.05) || (System.currentTimeMillis() > m_recordedTime + m_maxTime); // If the robot is within
        // 0.5 degrees of the
        // target and 0.05 meters
        // of the target, the
        // command is finished ||
        // m_trackingMode ==
        // "rotational"
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(0, 0, 0, true);
        m_isTimeRecorded = false;
    }
}