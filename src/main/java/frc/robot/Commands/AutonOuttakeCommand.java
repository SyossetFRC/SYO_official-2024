package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OuttakeSubsystem;

public class AutonOuttakeCommand extends Command {
    private final OuttakeSubsystem m_outtakeSubsystem;

    private final double m_outtakeRateSupplier;
    private final double m_outtakeAngle;

    private final long m_maxTime;
    private long m_recordedTime;
    private boolean m_isTimeRecorded;

    private final PIDController m_anglePIDController;

    /**
     * Command to engage the outtake using joystick input.
     * 
     * @param outtakeSubsystem The outtake subsystem.
     * @param outtakeRateSupplier The desired outtake rate (rpm).
     * @param outtakeAngle The desired outtake angle (rad).
     * @param maxTime The maximum time alloted for this command (ms).
     */
    public AutonOuttakeCommand(OuttakeSubsystem outtakeSubsystem, double outtakeRateSupplier, double outtakeAngle, long maxTime) {
        this.m_outtakeSubsystem = outtakeSubsystem;
        this.m_outtakeRateSupplier = outtakeRateSupplier;
        this.m_outtakeAngle = outtakeAngle;

        this.m_maxTime = maxTime;
        this.m_isTimeRecorded = false;

        m_anglePIDController = new PIDController(7.5, 0, 0);

        addRequirements(outtakeSubsystem);
    }

    /**
     * Command to engage the outtake using joystick input. Keeps outtake angle the same.
     * 
     * @param outtakeSubsystem The outtake subsystem.
     * @param outtakeRateSupplier The desired outtake rate (rpm).
     * @param maxTime The maximum time alloted for this command (ms).
     */
    public AutonOuttakeCommand(OuttakeSubsystem outtakeSubsystem, double outtakeRateSupplier, long maxTime) {
        this(outtakeSubsystem, outtakeRateSupplier, outtakeSubsystem.getAngle(), maxTime);
    }

    @Override
    public void execute() {
        if (!m_isTimeRecorded) {
            m_recordedTime = System.currentTimeMillis();
            m_isTimeRecorded = true;
        }

        m_outtakeSubsystem.outtake(m_outtakeRateSupplier);
        m_outtakeSubsystem.rotate(m_anglePIDController.calculate(m_outtakeSubsystem.getAngle(), m_outtakeAngle));
    }

    @Override
    public boolean isFinished() { return System.currentTimeMillis() > m_recordedTime + m_maxTime; }

    @Override
    public void end(boolean interrupted) { 
        m_outtakeSubsystem.outtake(0);
        m_outtakeSubsystem.rotate(0);
    }
}