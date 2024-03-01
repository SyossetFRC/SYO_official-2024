package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax m_climberMotor1;
    private final CANSparkMax m_climberMotor2;

    private double m_climberPower;

    public ClimberSubsystem() {
        m_climberMotor1 = new CANSparkMax(Constants.CLIMBER_MOTOR_1, MotorType.kBrushless);
        m_climberMotor2 = new CANSparkMax(Constants.CLIMBER_MOTOR_2, MotorType.kBrushless);

        m_climberMotor1.setIdleMode(IdleMode.kBrake);
        m_climberMotor2.setIdleMode(IdleMode.kBrake);

        m_climberMotor1.setInverted(false);
        m_climberMotor2.follow(m_climberMotor1, true);
    }

    /**
     * Engages the climber.
     * 
     * @param climberPower The power of the climber [-1, 1].
     */
    public void climb(double climberPower) {
        m_climberPower = climberPower;
    }

    @Override
    public void periodic() {
        m_climberMotor1.set(m_climberPower);
    }
}
