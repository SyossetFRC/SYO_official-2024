package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OuttakeSubsystem extends SubsystemBase {
    private static final double kOuttakeGearRatio = (1.0 / 3.0);
    public static final double kOuttakeMaxRate = 5676.0 * kOuttakeGearRatio; // rpm

    private final CANSparkMax m_outtakeMotor1;
    private final CANSparkMax m_outtakeMotor2;
    private final CANSparkMax m_linearActuator;

    private final RelativeEncoder m_outtakeEncoder;
    private final DutyCycleEncoder m_rotateEncoder;

    private final SimpleMotorFeedforward m_outtakeFeedforward = new SimpleMotorFeedforward(0, 0.00634);

    private final GenericEntry m_outtakeRateEntry;
    private final GenericEntry m_linearActuatorPositionEntry;

    private double m_outtakeRate;
    private double m_actuatorPower;

    public OuttakeSubsystem() {
        m_outtakeMotor1 = new CANSparkMax(Constants.OUTTAKE_MOTOR_1, MotorType.kBrushless);
        m_outtakeMotor2 = new CANSparkMax(Constants.OUTTAKE_MOTOR_2, MotorType.kBrushless);
        m_linearActuator = new CANSparkMax(Constants.LINEAR_ACTUATOR, MotorType.kBrushed);

        m_outtakeMotor1.setIdleMode(IdleMode.kCoast);
        m_outtakeMotor2.setIdleMode(IdleMode.kCoast);
        m_linearActuator.setIdleMode(IdleMode.kBrake);

        m_outtakeMotor1.setInverted(false);
        m_outtakeMotor2.follow(m_outtakeMotor1, false);
        m_linearActuator.setInverted(true);

        m_outtakeEncoder = m_outtakeMotor1.getEncoder();
        m_outtakeEncoder.setPositionConversionFactor(kOuttakeGearRatio); // rpm
        m_outtakeEncoder.setVelocityConversionFactor(kOuttakeGearRatio); // rpm
        m_rotateEncoder = new DutyCycleEncoder(Constants.OUTTAKE_ROTATE_ENCODER);

        ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
        ShuffleboardLayout outtakeLayout = tab.getLayout("Outtake", BuiltInLayouts.kList).withSize(2, 2).withPosition(2, 0);
        m_outtakeRateEntry = outtakeLayout.add("Outtake Rate", m_outtakeEncoder.getVelocity() + " rpm").getEntry();
        m_linearActuatorPositionEntry = outtakeLayout.add("Outtake Angle", getAngle() + " rad").getEntry();
    }

    /**
     * Engages the outtake.
     * 
     * @param outtakeRate The rate of outtake (rpm).
     */
    public void outtake(double outtakeRate) {
        m_outtakeRate = outtakeRate;
    }

    /**
     * Engages the actuator to rotate the outtake.
     * 
     * @param actuatorPower Linear actuator power [-1.0, 1.0].
     */
    public void rotate(double actuatorPower) {
        m_actuatorPower = actuatorPower;
    }

    /**
     * Returns the current angle of the outtake.
     * 
     * @return Angle of the outtake (rad).
     */
    public double getAngle() {
        return -2 * Math.PI * m_rotateEncoder.getAbsolutePosition();
    }

    /** Displays the periodically updated outtake rate on the Shuffleboard */
    public void updateShuffleboard() {
        m_outtakeRateEntry.setString(m_outtakeEncoder.getVelocity() + " rpm");
        m_linearActuatorPositionEntry.setString(getAngle() + " rad");
    }

    @Override
    public void periodic() {
        m_outtakeMotor1.setVoltage(m_outtakeFeedforward.calculate(m_outtakeRate));
        m_linearActuator.set(m_actuatorPower);
        updateShuffleboard();
    }
}
