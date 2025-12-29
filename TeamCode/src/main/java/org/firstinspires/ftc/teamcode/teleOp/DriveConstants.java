package org.firstinspires.ftc.teamcode.teleOp;

public class DriveConstants {

    // Physical constants
    public static final double WHEEL_RADIUS = 1.9685; // inches (50mm)
    public static final double GEAR_RATIO = 1.0;      // output (wheel) / input (motor)
    public static final double TRACK_WIDTH = 14.5;    // inches, distance between left/right wheels

    // Motor encoder
    public static final int MOTOR_TICKS_PER_REV = 537; // REV HD Hex 20:1

    // Feedforward parameters (tune these)
    public static final double kV = 0.018;  // velocity constant
    public static final double kA = 0.002;  // acceleration constant
    public static final double kStatic = 0.0; // static friction

    // Max robot velocity/acceleration (tune later)
    public static final double MAX_VEL = 60;   // inches per second
    public static final double MAX_ACCEL = 30; // inches per second^2
    public static final double MAX_ANG_VEL = Math.toRadians(180); // rad/s
    public static final double MAX_ANG_ACCEL = Math.toRadians(90); // rad/s^2

    // Utility
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / MOTOR_TICKS_PER_REV;
    }
}
