package frc.robot.Subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    private double[] m_limelightOdometry;
    private NetworkTableEntry m_angleX;

    private final GenericEntry m_distanceToNearestSpeakerEntry;
    private final GenericEntry m_outtakeAngleEntry;
    private final GenericEntry m_drivetrainAngleChangeEntry;

    public LimelightSubsystem() {
        NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        m_limelightOdometry = networkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        m_angleX = networkTable.getEntry("tx");

        ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
        ShuffleboardLayout limelightLayout = tab.getLayout("Limelight", BuiltInLayouts.kList).withSize(2, 3).withPosition(6, 0);
        m_distanceToNearestSpeakerEntry = limelightLayout.add("Distance to Nearest Speaker", getDistanceToNearestSpeaker() + " m").getEntry();
        m_outtakeAngleEntry = limelightLayout.add("Desired Outtake Angle", calculateOuttakeAngle() + " rad").getEntry();
        m_drivetrainAngleChangeEntry = limelightLayout.add("Desired Drivetrain Angle Change", getDrivetrainAngleChange() + " rad").getEntry();
    }

    @Override
    public void periodic() {
        m_distanceToNearestSpeakerEntry.setString(getDistanceToNearestSpeaker() + " m");
        m_outtakeAngleEntry.setString(calculateOuttakeAngle() + " rad");
        m_drivetrainAngleChangeEntry.setString(getDrivetrainAngleChange() + " rad");
    }

    private double getDistanceToNearestSpeaker() {
        double distance_blue = Math.sqrt(Math.pow(m_limelightOdometry[0] + 0.0381,2) + Math.pow(m_limelightOdometry[1] - 5.5479,2));
        double distance_red = Math.sqrt(Math.pow(m_limelightOdometry[0] - 16.579 ,2) + Math.pow(m_limelightOdometry[1] - 5.5479,2));
        if (distance_red < distance_blue)
        {
            return distance_red;
        }
        else if (distance_blue < distance_red)
        {
            return distance_blue;
        }
        return 1.33;
    }

    /**
     * Calculates the optimal outtake angle for shooting based on limelight trigonometric input.
     * 
     * @return Optimal outtake absolute angle (rad).
     */
    public double calculateOuttakeAngle() {
        return 0;
    }

    /**
     * Returns the change in drivetrain angle necessary for shooting based on limelight input.
     * 
     * @return Change in drivetrain angle (rad).
     */
    public double getDrivetrainAngleChange() {
        return -Math.toRadians(m_angleX.getDouble(0));
    }
}