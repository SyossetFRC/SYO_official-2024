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
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 9; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 14; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 50; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(175.6); 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 10; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 16; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 53; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(179.7); 
       
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 17; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 51; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = Math.toRadians(3.2); 

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 18; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 15; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 52; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(-2.8); 
}