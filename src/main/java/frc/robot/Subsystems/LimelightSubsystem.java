package frc.robot.Subsystems;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {

    private NetworkTable m_networkTable;

    private NetworkTableEntry m_pipelineId;
    private NetworkTableEntry m_camMode;
    private NetworkTableEntry m_ledMode;

    private NetworkTableEntry m_tx;
    private NetworkTableEntry m_ty;
    private NetworkTableEntry m_ta;
    private NetworkTableEntry m_tv;
    private NetworkTableEntry m_tid;

    private double[] m_botPose;
    private double[] m_targetPose;

    private double m_areaDistance;
    private double m_trigDistance;

    private double m_calculatedAngle;

    // All in inches, array index is tag ID - 1
    private final double[] m_tagHeight = new double[] {
            48.500 + 4.500, // Tag 1 (Source)
            48.500 + 4.500, // Tag 2 (Source)
            51.875 + 4.500, // Tag 3 (Speaker)
            51.875 + 4.500, // Tag 4 (Speaker)
            48.125 + 4.500, // Tag 5 (Amp)
            48.125 + 4.500, // Tag 6 (Amp)
            51.875 + 4.500, // Tag 7 (Speaker)
            51.875 + 4.500, // Tag 8 (Speaker)
            48.500 + 4.500, // Tag 9 (Source)
            48.500 + 4.500, // Tag 10 (Source)
            47.500 + 4.500, // Tag 11 (Stage)
            47.500 + 4.500, // Tag 12 (Stage)
            47.500 + 4.500, // Tag 13 (Stage)
            47.500 + 4.500, // Tag 14 (Stage)
            47.500 + 4.500, // Tag 15 (Stage)
            47.500 + 4.500 // Tag 16 (Stage)
    };

    // All in meters, rotation in radians, array index is tag ID - 1
    private Pose2d[] m_tagPose2d = new Pose2d[] {
            new Pose2d(15.23, 0.88, new Rotation2d(0.0)), // Tag 1 (Source)
            new Pose2d(15.91, 1.25, new Rotation2d(0.0)), // Tag 2 (Source)
            new Pose2d(16.50, 5.00, new Rotation2d(0.0)), // Tag 3 (Speaker)
            new Pose2d(16.30, 5.60, new Rotation2d(0.0)), // Tag 4 (Speaker)
            new Pose2d(14.70, 8.10, new Rotation2d(0.0)), // Tag 5 (Amp)
            new Pose2d(1.80, 8.20, new Rotation2d(0.0)), // Tag 6 (Amp)
            new Pose2d(0.65, 5.50, new Rotation2d(0.0)), // Tag 7 (Speaker)
            new Pose2d(0.00, 5.00, new Rotation2d(0.0)), // Tag 8 (Speaker)
            new Pose2d(0.65, 0.70, new Rotation2d(0.0)), // Tag 9 (Source)
            new Pose2d(1.20, 0.40, new Rotation2d(0.0)), // Tag 10 (Source)
            new Pose2d(12.00, 3.75, new Rotation2d(0.0)), // Tag 11 (Stage)
            new Pose2d(12.00, 4.50, new Rotation2d(0.0)), // Tag 12 (Stage)
            new Pose2d(11.25, 4.00, new Rotation2d(0.0)), // Tag 13 (Stage)
            new Pose2d(5.35, 4.00, new Rotation2d(0.0)), // Tag 14 (Stage)
            new Pose2d(4.65, 4.50, new Rotation2d(0.0)), // Tag 15 (Stage)
            new Pose2d(4.65, 3.75, new Rotation2d(0.0)), // Tag 16 (Stage)
    };

    private GenericEntry pipelineIdEntry;
    private GenericEntry camModeEntry;
    private GenericEntry ledModeEntry;
    private GenericEntry XEntry;
    private GenericEntry YEntry;
    private GenericEntry targetAreaEntry;
    private GenericEntry foundTagEntry;
    private GenericEntry tagIdEntry;
    private GenericEntry botPoseEntry;
    private GenericEntry targetPoseEntry;
    private GenericEntry areaDistanceEntry;
    private GenericEntry trigDistanceEntry;
    private GenericEntry distanceToTagEntry;

    SendableChooser<String> trackingModeChooser = new SendableChooser<>();

    public LimelightSubsystem() {
        m_networkTable = NetworkTableInstance.getDefault().getTable("limelight");

        m_pipelineId = m_networkTable.getEntry("getpipe"); // Active pipeline index of the camera (0-9)

        m_camMode = m_networkTable.getEntry("camMode"); // Vision processing mode (0) or driver camera mode (1)
                                                        // (increases exposure, disables vision processing)
        m_ledMode = m_networkTable.getEntry("ledMode"); // Current LED mode of the camera (0-pipeline default, 1-off,
                                                        // 2-blink, 3-on)

        m_tx = m_networkTable.getEntry("tx"); // Horizontal offset from crosshair to target (-29.8 to 29.8 degrees)
        m_ty = m_networkTable.getEntry("ty"); // Vertical offset from crosshair to target (-24.85 to 24.85 degrees)
        m_ta = m_networkTable.getEntry("ta"); // Target area (0% of image to 100% of image)
        m_tv = m_networkTable.getEntry("tv"); // Any valid targets (0 or 1 (found))
        m_tid = m_networkTable.getEntry("tid"); // Target april tag ID (0, 1, 2, 3, 4, 5)

        // Needs testing!
        m_botPose = m_networkTable.getEntry("botpose").getDoubleArray(new double[6]); // Pose of the robot in the world
                                                                                      // frame (x, y, z, pitch, yaw,
                                                                                      // roll)
        m_targetPose = m_networkTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); // Pose of the
                                                                                                        // target in the
                                                                                                        // camera frame
                                                                                                        // (x, y, z,
                                                                                                        // pitch, yaw,
                                                                                                        // roll)

        ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");

        pipelineIdEntry = limelightTab.add("Pipeline ID", 0).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 9)).getEntry();
        // camModeEntry = limelightTab.add("Camera Mode",
        // 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        // ledModeEntry = limelightTab.add("LED Mode",
        // 0).withWidget(BuiltInWidgets.kToggleButton).getEntry();

        ShuffleboardLayout limelightDataLayout = Shuffleboard.getTab("Limelight")
                .getLayout("Limelight Data", BuiltInLayouts.kList).withSize(2, 3);
        XEntry = limelightDataLayout.add("X Angle", 0).getEntry();
        YEntry = limelightDataLayout.add("Y Angle", 0).getEntry();
        targetAreaEntry = limelightDataLayout.add("Target Area", 0).getEntry();
        foundTagEntry = limelightDataLayout.add("Found Tag", 0).getEntry();
        tagIdEntry = limelightDataLayout.add("Tag ID", 0).getEntry();
        botPoseEntry = limelightDataLayout.add("Bot Pose", 0).getEntry();
        targetPoseEntry = limelightDataLayout.add("Target Pose", 0).getEntry();
        areaDistanceEntry = limelightDataLayout.add("Area Distance", 0).getEntry();
        trigDistanceEntry = limelightDataLayout.add("Trig Distance", 0).getEntry();

        ShuffleboardLayout tagTrackingLayout = Shuffleboard.getTab("Limelight")
                .getLayout("Tag Tracking Variables", BuiltInLayouts.kList).withSize(2, 2);
        trackingModeChooser = new SendableChooser<>();
        trackingModeChooser.setDefaultOption("Translational", "translational");
        trackingModeChooser.addOption("Rotational", "rotational");
        tagTrackingLayout.add(trackingModeChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 1);
        distanceToTagEntry = tagTrackingLayout.add("Distance to Tag (Meters)", 0.80)
                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
    }

    @Override
    public void periodic() {
        m_areaDistance = Units.inchesToMeters(54.4 * Math.pow(m_ta.getDouble(0), -0.475)); // Calculates distance based
                                                                                           // on graphed ta values, used
                                                                                           // google sheets to calculate
                                                                                           // curve
        m_trigDistance = Units.inchesToMeters(51.96 -
        Constants.LIMELIGHT_LENS_HEIGHT) / Math.tan(Math.toRadians(m_ty.getDouble(0.0) + Constants.LIMELIGHT_ANGLE)); // Calculates distance using trigonometry. Reference a triangle and the notion that tan(theta) = opposite/adjacent. Opposite height of target - height of camera, adjacent = distance from camera to target, theta = angle of camera to target. Rearrange to get d = (h2-h1) / tan(a1+a2)


        
        m_pipelineId.setNumber(pipelineIdEntry.getDouble(0));
        // m_camMode.setNumber(camModeEntry.getBoolean(false) ? 1 : 0);
        // m_ledMode.setNumber(ledModeEntry.getBoolean(false) ? 3 : 1);

        XEntry.setDouble(m_tx.getDouble(0));
        YEntry.setDouble(m_ty.getDouble(0));
        targetAreaEntry.setDouble(m_ta.getDouble(0));
        foundTagEntry.setDouble(m_tv.getDouble(0));
        tagIdEntry.setDouble(m_tid.getDouble(0));
        botPoseEntry.setDoubleArray(m_botPose);
        targetPoseEntry.setDoubleArray(m_targetPose);
        areaDistanceEntry.setDouble(m_areaDistance);
        trigDistanceEntry.setDouble(m_trigDistance);
    }

    public double getDistance(String mode) {
        if (mode.equals("Area")) {
            return m_areaDistance;
        } else if (mode.equals("Trig")) {
            return m_trigDistance;
        } else {
            return 0.0;
        }
    }

    /**
     * Calculates the optimal outtake angle for shooting based on limelight trigonometric input.
     * 
     * @return Optimal outtake absolute angle (rad).
     */
    public double calculateOuttakeAngle() {
        return -0.190354293083 * getDistance("Trig") - 1.930509;
    }

    /**
     * Returns the change in drivetrain angle necessary for shooting based on limelight input.
     * 
     * @return Change in drivetrain angle (rad).
     */
    public double getDrivetrainAngleChange() {
        return -Math.toRadians(m_tx.getDouble(0));
    }

    public String getTrackingMode() {
        return trackingModeChooser.getSelected();
    }

    public double getDistanceToTag() {
        return distanceToTagEntry.getDouble(0.80);
    }

    public boolean getTagFound() {
        return m_tv.getDouble(0) != 0;
    }

    /*
    public double getTagID() {
        return m_tid.getDouble(0.0);
    }

    public Pose2d getTagPose2d(int tagID) {
        return m_tagPose2d[tagID - 1];
    }
    */
}