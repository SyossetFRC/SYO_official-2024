package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private static final double kIntakeGearRatio = (1.0 / 4.0);
    public static final double kIntakeMaxRate = 5676.0 * kIntakeGearRatio; // rpm

    private static final double kRotateGearRatio = (1.0 / 100.0) * (60.0 / 64.0);
    public static final double kRotateMaxAngularSpeed = 5676.0 * kRotateGearRatio * 2.0 * Math.PI / 60; // rad/s

    private final CANSparkMax m_intakeMotor;
    private final CANSparkMax m_rotateMotor;

    private final RelativeEncoder m_intakeEncoder;
    private final RelativeEncoder m_rotateEncoder;

    private final DigitalInput m_lowLimitSwitch;
    private final DigitalInput m_highLimitSwitch;
    private final DigitalInput m_noteLimitSwitch;

    private final SimpleMotorFeedforward m_intakeFeedforward = new SimpleMotorFeedforward(0, 0.00845);
    private final SimpleMotorFeedforward m_rotateFeedforward = new SimpleMotorFeedforward(0, 2.15);

    private final AddressableLED m_Led;
    private final AddressableLEDBuffer m_LedBuffer;

    private final GenericEntry m_intakeRateEntry;
    private final GenericEntry m_rotateAngleEntry;
    private final GenericEntry m_rotateAngularSpeedEntry;
    private final GenericEntry m_lowLimitSwitchEntry;
    private final GenericEntry m_highLimitSwitchEntry;
    private final GenericEntry m_noteLimitSwitchEntry;

    private double m_intakeRate;
    private double m_angularSpeed;

    public IntakeSubsystem() {
        m_intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);
        m_rotateMotor = new CANSparkMax(Constants.ROTATE_MOTOR, MotorType.kBrushless);

        m_intakeMotor.setIdleMode(IdleMode.kBrake);
        m_rotateMotor.setIdleMode(IdleMode.kBrake);

        m_intakeMotor.setInverted(false);
        m_rotateMotor.setInverted(true);

        m_intakeEncoder = m_intakeMotor.getEncoder();
        m_intakeEncoder.setPositionConversionFactor(kIntakeGearRatio); // rot
        m_intakeEncoder.setVelocityConversionFactor(kIntakeGearRatio); // rpm

        m_rotateEncoder = m_rotateMotor.getEncoder();
        m_rotateEncoder.setPositionConversionFactor(kRotateGearRatio * 2.0 * Math.PI); // rad
        m_rotateEncoder.setVelocityConversionFactor(kRotateGearRatio * 2.0 * Math.PI / 60); // rad/s

        m_lowLimitSwitch = new DigitalInput(Constants.LOW_LIMIT_SWITCH);
        m_highLimitSwitch = new DigitalInput(Constants.HIGH_LIMIT_SWITCH);
        m_noteLimitSwitch = new DigitalInput(Constants.NOTE_LIMIT_SWITCH);

        m_Led = new AddressableLED(1);
        m_LedBuffer = new AddressableLEDBuffer(144);
        m_Led.setLength(m_LedBuffer.getLength());
        m_Led.setData(m_LedBuffer);
        m_Led.start();

        ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
        ShuffleboardLayout intakeLayout = tab.getLayout("Intake", BuiltInLayouts.kList).withSize(2, 6).withPosition(0, 0);
        m_intakeRateEntry = intakeLayout.add("Intake Rate", m_intakeEncoder.getVelocity() + " rpm").getEntry();
        m_rotateAngleEntry = intakeLayout.add("Intake Angle", m_rotateEncoder.getPosition() + " rad").getEntry();
        m_rotateAngularSpeedEntry = intakeLayout.add("Intake Angular Speed", m_rotateEncoder.getVelocity() + " rad/s").getEntry();
        m_lowLimitSwitchEntry = intakeLayout.add("Low Limit Switch", !m_lowLimitSwitch.get()).getEntry();
        m_highLimitSwitchEntry = intakeLayout.add("High Limit Switch", !m_highLimitSwitch.get()).getEntry();
        m_noteLimitSwitchEntry = intakeLayout.add("Note Limit Switch", !m_noteLimitSwitch.get()).getEntry();
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
        m_intakeRateEntry.setString(m_intakeEncoder.getVelocity() + " rpm");
        m_rotateAngleEntry.setString(m_rotateEncoder.getPosition() + " rad");
        m_rotateAngularSpeedEntry.setString(m_rotateEncoder.getVelocity() + " rad/s");
        m_lowLimitSwitchEntry.setBoolean(!m_lowLimitSwitch.get());
        m_highLimitSwitchEntry.setBoolean(!m_highLimitSwitch.get());
        m_noteLimitSwitchEntry.setBoolean(!m_noteLimitSwitch.get());
    }

    /** Updates the LED colors depending on whether the note is intaked. Red is false and green is true. */
    public void updateLEDs() {
        if (m_noteLimitSwitch.get()){
            for (var i = 0; i < m_LedBuffer.getLength(); i++)
            {
              m_LedBuffer.setRGB(i, 255, 0, 0);
            }
        }
        else {
            for (var i = 0; i < m_LedBuffer.getLength(); i++)
            {
                m_LedBuffer.setRGB(i, 0, 255, 0);
            }
        }
        m_Led.setData(m_LedBuffer);
    }

    @Override
    public void periodic() {
        if (canIntake()) {
            m_intakeMotor.setVoltage(m_intakeFeedforward.calculate(m_intakeRate));
        } else {
            m_intakeMotor.setVoltage(0);
        }
        if (canRotate()) {
            m_rotateMotor.setVoltage(m_rotateFeedforward.calculate(m_angularSpeed));
        } else {
            m_rotateMotor.setVoltage(0);
        }
        updateLEDs();
        updateShuffleboard();
    }

    /** Returns whether the intake can be activated. */
    public boolean canIntake() {
        return true;
    }

    /** Returns whether the intake can rotate. */
    public boolean canRotate() {
        if (m_angularSpeed > 0 && !m_highLimitSwitch.get()) {
            return false;
        }
        if (m_angularSpeed < 0 && !m_lowLimitSwitch.get()) {
            return false;
        }
        return true;
    }
}
