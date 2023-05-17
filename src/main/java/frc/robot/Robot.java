package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  RobotContainer m_container;

  private double m_startXPos;
  private double m_startYPos;
  private double m_startTheta;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() { m_container = new RobotContainer(); }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() { CommandScheduler.getInstance().run(); }

  @Override
  public void autonomousInit() {
    CommandScheduler.getInstance().cancelAll();

    m_container.setPose(m_startXPos, m_startYPos, m_startTheta);
    m_container.setIdleMode("brake");

    m_container.autonomousCommands().schedule();
  }

  @Override
  public void teleopInit() {
    CommandScheduler.getInstance().cancelAll();
    
    m_container.setIdleMode("brake");
  }
}