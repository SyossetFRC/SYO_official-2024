package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  RobotContainer m_container;

  private double m_startX;
  private double m_startY;
  private double m_startTheta;
  private ArrayList<SpikeMarkNote> m_autonomousNotes = new ArrayList<>();

  private ShuffleboardTab m_tab;
  private ShuffleboardLayout m_startPositionLayout;
  private ShuffleboardLayout m_noteChooserLayout;

  private GenericEntry m_startXEntry;
  private GenericEntry m_startYEntry;
  private GenericEntry m_startThetaEntry;
  private GenericEntry m_startPositionOutputEntry;
  private GenericEntry m_leftNoteButton;
  private GenericEntry m_middleNoteButton;
  private GenericEntry m_rightNoteButton;
  private GenericEntry m_autonomousNotesOutputEntry;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() { 
    m_container = new RobotContainer(); 

    m_tab = Shuffleboard.getTab("Autonomous");

    m_startPositionLayout = m_tab.getLayout("Starting Position", BuiltInLayouts.kList).withSize(2, 5).withPosition(0, 0);
    m_startXEntry = m_startPositionLayout.add("Input Starting X Position (m)", 0).getEntry();
    m_startYEntry = m_startPositionLayout.add("Input Starting Y Position (m)", 0).getEntry();
    m_startThetaEntry = m_startPositionLayout.add("Input Starting Angle (deg)", 0).getEntry();
    m_startPositionOutputEntry = m_startPositionLayout.add("Starting Position", "(0, 0, 0)").getEntry();
    m_startPositionLayout.add("Instructions", "The origin is the center of the front subwoofer edge. The +x direction is forward or away from the driver station. From the POV of the driver station, the +y direction is left. Counterclocksize is +theta. Measure starting position from the origin to the center of the robot, in meters and degrees.");

    m_noteChooserLayout = m_tab.getLayout("Notes", BuiltInLayouts.kList).withSize(2, 5).withPosition(2, 0);
    m_leftNoteButton = m_noteChooserLayout.add("Left Note", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    m_middleNoteButton = m_noteChooserLayout.add("Middle Note", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    m_rightNoteButton = m_noteChooserLayout.add("Right Note", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
    m_autonomousNotesOutputEntry = m_noteChooserLayout.add("Notes", "{}").getEntry();
    m_noteChooserLayout.add("Instructions", "Select the buttons in the order that the robot will intake/outtake them. Deselect to remove.");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() { 
    CommandScheduler.getInstance().run(); 

    m_startX = m_startXEntry.getDouble(0);
    m_startY = m_startYEntry.getDouble(0);
    m_startTheta = Math.toRadians(m_startThetaEntry.getDouble(0));
    m_startPositionOutputEntry.setString("(" + m_startX + ", " + m_startY + ", " + m_startTheta + ")");

    if (m_leftNoteButton.getBoolean(false)) {
      if (!m_autonomousNotes.contains(SpikeMarkNote.LEFT)) {
        m_autonomousNotes.add(SpikeMarkNote.LEFT);
      }
    } else {
      m_autonomousNotes.remove(SpikeMarkNote.LEFT);
    }
    if (m_middleNoteButton.getBoolean(false)) {
      if (!m_autonomousNotes.contains(SpikeMarkNote.MIDDLE)) {
        m_autonomousNotes.add(SpikeMarkNote.MIDDLE);
      }
    } else {
      m_autonomousNotes.remove(SpikeMarkNote.MIDDLE);
    }
    if (m_rightNoteButton.getBoolean(false)) {
      if (!m_autonomousNotes.contains(SpikeMarkNote.RIGHT)) {
        m_autonomousNotes.add(SpikeMarkNote.RIGHT);
      }
    } else {
      m_autonomousNotes.remove(SpikeMarkNote.RIGHT);
    }
    m_autonomousNotesOutputEntry.setString(printAutonomousNotes());
  }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();

    m_container.autonomousCommands(m_startX, m_startY, m_startTheta, m_autonomousNotes).schedule();
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  private String printAutonomousNotes() {
    String autonomousNotes = "{";
    for (SpikeMarkNote note : m_autonomousNotes) {
      switch(note) {
        case LEFT:
          autonomousNotes += " LEFT ";
          break;
        case MIDDLE:
          autonomousNotes += " MIDDLE ";
          break;
        case RIGHT:
          autonomousNotes += " RIGHT ";
          break;
      }
    }
    autonomousNotes += "}";
    return autonomousNotes;
  }
}