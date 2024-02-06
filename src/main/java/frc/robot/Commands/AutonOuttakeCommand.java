package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OuttakeSubsystem;

public class AutonOuttakeCommand extends Command {
    private final OuttakeSubsystem m_outtakeSubsystem;

    private final double m_outtakeRateSupplier;

    private final long m_maxTime;
    private long m_recordedTime;
    private boolean m_isTimeRecorded;

    /**
     * Command to engage the outtake using joystick input.
     * 
     * @param outtakeSubsystem The outtake subsystem.
     * @param outtakeRateSupplier The desired outtake rate (rpm).
     * @param maxTime The maximum time alloted for this command (ms).
     */
    public AutonOuttakeCommand(OuttakeSubsystem outtakeSubsystem, double outtakeRateSupplier, long maxTime) {
        this.m_outtakeSubsystem = outtakeSubsystem;
        this.m_outtakeRateSupplier = outtakeRateSupplier;

        this.m_maxTime = maxTime;
        this.m_isTimeRecorded = false;

        addRequirements(outtakeSubsystem);
    }

    @Override
    public void execute() {
        if (!m_isTimeRecorded) {
            m_recordedTime = System.currentTimeMillis();
            m_isTimeRecorded = true;
        }

        m_outtakeSubsystem.outtake(m_outtakeRateSupplier);
    }

    @Override
    public boolean isFinished() { return System.currentTimeMillis() > m_recordedTime + m_maxTime; }

    @Override
    public void end(boolean interrupted) { 
        m_outtakeSubsystem.outtake(0);
    }
}