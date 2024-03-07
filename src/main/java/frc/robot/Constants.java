package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 53;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -147.4 / 360.0; // Input between -180 and 180 degrees.

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 52;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 60.2 / 360.0; // Input between -180 and 180 degrees.
       
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 9;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 50;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 25 / 360.0; // Input between -180 and 180 degrees.

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 5;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 51;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -154.7 / 360.0; // Input between -180 and 180 degrees.

    public static final int INTAKE_MOTOR = 24;
    public static final int ROTATE_MOTOR = 41;
    public static final int LOW_LIMIT_SWITCH = 4;
    public static final int HIGH_LIMIT_SWITCH = 6;

    public static final int OUTTAKE_MOTOR_1 = 60;
    public static final int OUTTAKE_MOTOR_2 = 23;
    public static final int LINEAR_ACTUATOR = 55;
    public static final int OUTTAKE_ROTATE_ENCODER = 5;

    public static final int CLIMBER_MOTOR_LEFT = 10;
    public static final int CLIMBER_MOTOR_RIGHT = 14;

    public static final double LIMELIGHT_LENS_HEIGHT = 27.00; // inches
    public static final double LIMELIGHT_ANGLE = 30.0; // degrees
}