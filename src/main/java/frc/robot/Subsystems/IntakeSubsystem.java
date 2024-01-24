package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax m_leftPivotMotor;
    private CANSparkMax m_rightPivotMotor;

    private CANSparkMax m_leftIntakeMotor;
    private CANSparkMax m_rightIntakeMotor;

    private double m_pivotSpeed = 0;
    
    public IntakeSubsystem() {
        m_leftPivotMotor = new CANSparkMax (Constants.LEFT_PIVOT_MOTOR, MotorType.kBrushless);
        m_rightPivotMotor = new CANSparkMax (Constants.RIGHT_PIVOT_MOTOR, MotorType.kBrushless);
        m_leftIntakeMotor = new CANSparkMax (Constants.LEFT_INTAKE_MOTOR, MotorType.kBrushless);
        m_rightIntakeMotor = new CANSparkMax (Constants.RIGHT_INTAKE_MOTOR, MotorType.kBrushless);

        m_leftPivotMotor.setIdleMode(IdleMode.kBrake);
        m_rightPivotMotor.setIdleMode(IdleMode.kBrake);
        m_leftIntakeMotor.setIdleMode(IdleMode.kBrake);
        m_rightIntakeMotor.setIdleMode(IdleMode.kBrake);
    }

    public void rotate(double pivotSpeed) {
        m_pivotSpeed = pivotSpeed;
    }

    public void intake() {
        m_leftIntakeMotor.set(-0.5);
        m_rightIntakeMotor.set(-0.5);
    }

    public void outtake() {
        m_leftIntakeMotor.set(0.5);
        m_rightIntakeMotor.set(0.5);
    }

    public void stop() {
        m_leftIntakeMotor.set(0);
        m_rightIntakeMotor.set(0);
    }

    @Override
    public void periodic() {
        m_leftPivotMotor.set(m_pivotSpeed);
        m_rightPivotMotor.set(m_pivotSpeed);
    }
}
