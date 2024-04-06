package frc.robot.Subsystems;

import java.util.concurrent.ArrayBlockingQueue;
import java.util.Collections;
import frc.robot.Subsystems.LimelightHelpers;
import frc.robot.Subsystems.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.geometry.Pose2d;
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
    NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");

    private double[] m_limelightodometry = networkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

    private final GenericEntry m_distanceToNearestSpeakerEntry;
    private final GenericEntry m_outtakeAngleEntry;
    private final GenericEntry m_drivetrainAngleChangeEntry;
    private final GenericEntry m_robotPose; 

    public LimelightSubsystem() {
        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        m_limelightodometry = networkTable.getEntry("botpose").getDoubleArray(new double[6]);

        m_limelight_x_position = new ArrayBlockingQueue<>(5, true, Collections.nCopies(5, m_limelightodometry[0]));
        m_limelight_y_position = new ArrayBlockingQueue<>(5, true, Collections.nCopies(5, m_limelightodometry[1]));
       
        m_angleX = networkTable.getEntry("tx");

        ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
        ShuffleboardLayout limelightLayout = tab.getLayout("Limelight", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0);
        m_distanceToNearestSpeakerEntry = limelightLayout.add("Distance to Nearest Speaker", getDistanceToNearestSpeaker() + " m").getEntry();
        m_outtakeAngleEntry = limelightLayout.add("Desired Outtake Angle", calculateOuttakeAngle() + " rad").getEntry();
        m_drivetrainAngleChangeEntry = limelightLayout.add("Desired Drivetrain Angle Change", getDrivetrainAngleChange() + " rad").getEntry();
        m_robotPose = limelightLayout.add("Limelight Robot Pose", "(" + m_limelightodometry[0] + ", " + m_limelightodometry[1] + ")").getEntry(); 
    }

    @Override
    public void periodic() {

        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        


        
        m_limelight_x_position.poll();
        m_limelight_x_position.add(mt2.pose.getX());
        m_limelight_y_position.poll();
        m_limelight_y_position.add(mt2.pose.getY());

        m_distanceToNearestSpeakerEntry.setString(getDistanceToNearestSpeaker() + " m");
        m_outtakeAngleEntry.setString(calculateOuttakeAngle() + " rad");
        m_drivetrainAngleChangeEntry.setString(getDrivetrainAngleChange() + " rad");
        m_robotPose.setString("(" + m_limelightodometry[0] + ", " + m_limelightodometry[1] + ")");
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
        return 1.3;
    }

    /**
     * Calculates the optimal outtake angle for shooting based on limelight trigonometric input.
     * 
     * @return Optimal outtake absolute angle (rad).
     */
    public double calculateOuttakeAngle() {
        
        return 1.27341 * Math.pow(.546087, getDistanceToNearestSpeaker()) - 3.67942;
    
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