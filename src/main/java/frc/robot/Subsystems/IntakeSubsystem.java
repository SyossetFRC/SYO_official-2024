package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private static final double kIntakeGearRatio = 0.05;
    public static final double kIntakeMaxRate = 5676.0 * kIntakeGearRatio; // rpm

    private static final double kRotateGearRatio = 0.05;
    public static final double kRotateMaxAngularSpeed = 5676.0 * kRotateGearRatio * 2.0 * Math.PI / 60; // rad/s

    private final CANSparkMax m_intakeMotor;
    private final CANSparkMax m_rotateMotor;

    private final RelativeEncoder m_intakeEncoder;
    private final RelativeEncoder m_rotateEncoder;

    private final PIDController m_intakePIDController = new PIDController(0.1, 0, 0);
    private final SimpleMotorFeedforward m_intakeFeedforward = new SimpleMotorFeedforward(0, 0.05132);

    private final GenericEntry m_intakeRateEntry;
    private final GenericEntry m_rotateAngleEntry;
    private final GenericEntry m_rotateAngularSpeedEntry;

    private double m_intakeRate;
    private double m_angularSpeed;

    public IntakeSubsystem() {
        m_intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
        m_rotateMotor = new CANSparkMax(Constants.ROTATE_MOTOR, MotorType.kBrushless);

        m_intakeMotor.setIdleMode(IdleMode.kBrake);
        m_rotateMotor.setIdleMode(IdleMode.kBrake);

        m_intakeEncoder = m_intakeMotor.getEncoder();
        m_intakeEncoder.setPositionConversionFactor(kIntakeGearRatio);
        m_intakeEncoder.setVelocityConversionFactor(kIntakeGearRatio);

        m_rotateEncoder = m_rotateMotor.getEncoder();
        m_rotateEncoder.setPositionConversionFactor(kRotateGearRatio * 2.0 * Math.PI);
        m_rotateEncoder.setVelocityConversionFactor(kRotateGearRatio * 2.0 * Math.PI / 60);

        ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
        ShuffleboardLayout intakeLayout = tab.getLayout("Intake", BuiltInLayouts.kList).withSize(2, 3).withPosition(0, 0);
        m_intakeRateEntry = intakeLayout.add("Intake Rate", m_intakeEncoder.getVelocity()).getEntry();
        m_rotateAngleEntry = intakeLayout.add("Intake Angle", m_rotateEncoder.getPosition()).getEntry();
        m_rotateAngularSpeedEntry = intakeLayout.add("Intake Angular Speed", m_rotateEncoder.getVelocity()).getEntry();
    }

    /**
     * Engages the intake.
     * 
     * @param intakeRate The rate of intake (rpm).
     */
    public void intake(double intakeRate) {
        m_intakeRate = intakeRate;
    }

    /**
     * Changes the intake angle.
     * 
     * @param angularSpeed The speed of angle change (rad/s).
     */
    public void rotate(double angularSpeed) {
        m_angularSpeed = angularSpeed;
    }

    /** Returns the current angle of the intake (rad). */
    public double getAngle() {
        return m_rotateEncoder.getPosition();
    }

    /** Resets the angle of the intake to 0. */
    public void reset() {
        m_rotateEncoder.setPosition(0);
    }

    /** Displays the periodically updated intake rate on the Shuffleboard */
    public void updateShuffleboard() {
        m_intakeRateEntry.setDouble(m_intakeEncoder.getVelocity());
        m_rotateAngleEntry.setDouble(m_rotateEncoder.getPosition());
        m_rotateAngularSpeedEntry.setDouble(m_rotateEncoder.getVelocity());
    }

    @Override
    public void periodic() {
        m_intakeMotor.setVoltage(m_intakePIDController.calculate(m_intakeEncoder.getVelocity(), m_intakeRate) + m_intakeFeedforward.calculate(m_intakeRate));
        m_rotateMotor.set(m_angularSpeed / kRotateMaxAngularSpeed);
        updateShuffleboard();
    }
}
