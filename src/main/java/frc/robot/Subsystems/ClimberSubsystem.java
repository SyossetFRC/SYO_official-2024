package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_climberMotorLeft;
    private final CANSparkMax m_climberMotorRight;

    private double m_climberPowerLeft;
    private double m_climberPowerRight;

    private final GenericEntry m_climberPowerLeftEntry;
    private final GenericEntry m_climberPowerRightEntry;

    public ClimberSubsystem() {
        m_climberMotorLeft = new CANSparkMax(Constants.CLIMBER_MOTOR_LEFT, MotorType.kBrushless);
        m_climberMotorRight = new CANSparkMax(Constants.CLIMBER_MOTOR_RIGHT, MotorType.kBrushless);

        m_climberMotorLeft.setIdleMode(IdleMode.kBrake);
        m_climberMotorRight.setIdleMode(IdleMode.kBrake);

        m_climberMotorLeft.setInverted(false);
        m_climberMotorRight.setInverted(true);

        ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
        ShuffleboardLayout climberLayout = tab.getLayout("Climber", BuiltInLayouts.kList).withSize(2, 2).withPosition(4, 0);
        m_climberPowerLeftEntry = climberLayout.add("Left Climber Power", m_climberPowerLeft).getEntry();
        m_climberPowerRightEntry = climberLayout.add("Right Climber Power", m_climberPowerRight).getEntry();
    }

    /**
     * Engages the climber.
     * 
     * @param climberPowerLeft The power of the left climber [-1, 1].
     * @param climberPowerRight The power of the right climber [-1, 1].
     */
    public void climb(double climberPowerLeft, double climberPowerRight) {
        m_climberPowerLeft = climberPowerLeft;
        m_climberPowerRight = climberPowerRight;
    }

    @Override
    public void periodic() {
        m_climberMotorLeft.set(m_climberPowerLeft);
        m_climberMotorRight.set(m_climberPowerRight);

        m_climberPowerLeftEntry.setDouble(m_climberPowerLeft);
        m_climberPowerRightEntry.setDouble(m_climberPowerRight);
    }
}
