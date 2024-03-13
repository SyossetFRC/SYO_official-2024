package frc.robot.Subsystems;

import java.util.concurrent.ArrayBlockingQueue;
import java.text.CollationElementIterator;
import java.util.Collections;

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
    private ArrayBlockingQueue<Double> m_limelight_x_position, m_limelight_y_position;
    private NetworkTableEntry m_angleX;

    private final GenericEntry m_distanceToNearestSpeakerEntry;
    private final GenericEntry m_outtakeAngleEntry;
    private final GenericEntry m_drivetrainAngleChangeEntry;

    public LimelightSubsystem() {
        NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        double[] limelightOdometry = networkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

        m_limelight_x_position = new ArrayBlockingQueue<>(5, true, Collections.nCopies(5, limelightOdometry[0]));
        m_limelight_y_position = new ArrayBlockingQueue<>(5, true, Collections.nCopies(5, limelightOdometry[1]));
       
        m_angleX = networkTable.getEntry("tx");

        ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
        ShuffleboardLayout limelightLayout = tab.getLayout("Limelight", BuiltInLayouts.kList).withSize(2, 3).withPosition(6, 0);
        m_distanceToNearestSpeakerEntry = limelightLayout.add("Distance to Nearest Speaker", getDistanceToNearestSpeaker() + " m").getEntry();
        m_outtakeAngleEntry = limelightLayout.add("Desired Outtake Angle", calculateOuttakeAngle() + " rad").getEntry();
        m_drivetrainAngleChangeEntry = limelightLayout.add("Desired Drivetrain Angle Change", getDrivetrainAngleChange() + " rad").getEntry();
    }

    @Override
    public void periodic() {
        NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        double[] limelightOdometry = networkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        m_limelight_x_position.poll();
        m_limelight_x_position.add(limelightOdometry[0]);
        m_limelight_y_position.poll();
        m_limelight_y_position.add(limelightOdometry[1]);

        m_distanceToNearestSpeakerEntry.setString(getDistanceToNearestSpeaker() + " m");
        m_outtakeAngleEntry.setString(calculateOuttakeAngle() + " rad");
        m_drivetrainAngleChangeEntry.setString(getDrivetrainAngleChange() + " rad");
    }

    private double getDistanceToNearestSpeaker() {
        double x_pos = 0, y_pos = 0;
        for (double d : m_limelight_x_position) x_pos += d;
        for (double d : m_limelight_y_position) y_pos += d;
        
        x_pos /= 5.0;
        y_pos /= 5.0;

        double distance_blue = Math.sqrt(Math.pow(x_pos + 0.0381,2) + Math.pow(y_pos - 5.5479,2));
        double distance_red = Math.sqrt(Math.pow(x_pos - 16.579 ,2) + Math.pow(y_pos - 5.5479,2));
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
        return .824959 * Math.pow(getDistanceToNearestSpeaker(),-.922543)-2.626262;
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