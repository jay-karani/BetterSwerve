package org.firstinspires.ftc.teamcode.swerveDrive;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Simple constants for swerve drive.
 * Everything in radians and meters.
 */
@Configurable
public class SwerveConstantsOld {

    // ==================== HARDWARE IDs ====================
    public static final String FRONT_LEFT_DRIVE = "fl_motor";
    public static final String FRONT_RIGHT_DRIVE = "fr_motor";
    public static final String BACK_LEFT_DRIVE = "bl_motor";
    public static final String BACK_RIGHT_DRIVE = "br_motor";

    public static final String FRONT_LEFT_STEER = "fl_rotation";
    public static final String FRONT_RIGHT_STEER = "fr_rotation";
    public static final String BACK_LEFT_STEER = "bl_rotation";
    public static final String BACK_RIGHT_STEER = "br_rotation";

    // Analog encoder IDs (built into CRServos)
    public static final String FRONT_LEFT_ENCODER = "fl_absolute";
    public static final String FRONT_RIGHT_ENCODER = "fr_absolute";
    public static final String BACK_LEFT_ENCODER = "bl_absolute";
    public static final String BACK_RIGHT_ENCODER = "br_absolute";


    // ==================== PHYSICAL MEASUREMENTS ====================
    // All distances in METERS
    public static final double WHEEL_BASE = 0.2159; // 12 inches = 0.3048 meters
    public static final double TRACK_WIDTH = 0.2159; // 12 inches = 0.3048 meters


    // ==================== SERVO CONFIGURATION ====================
    // Analog voltage range (typically 3.3V for FTC)
    public static final double ANALOG_VOLTAGE_RANGE = 3.1865;

    // Encoder offsets in RADIANS (calibrate these!)
    public static final double FRONT_LEFT_OFFSET = 6.0466;
    public static final double FRONT_RIGHT_OFFSET = 1.332;
    public static final double BACK_LEFT_OFFSET = 4.7116;
    public static final double BACK_RIGHT_OFFSET = 1.9984;

    // PIDF for steering servos (tune these!)
    public static PIDFCoefficients STEER_PIDF = new PIDFCoefficients(
            0.02,   // P
            0.0,   // I
            0.0,  // D
            0.0    // F
    );


    // ==================== MOTOR CONFIGURATION ====================
    // Invert motors if needed (true = reversed)
    public static final boolean FRONT_LEFT_DRIVE_INVERTED = false;
    public static final boolean FRONT_RIGHT_DRIVE_INVERTED = false;
    public static final boolean BACK_LEFT_DRIVE_INVERTED = false;
    public static final boolean BACK_RIGHT_DRIVE_INVERTED = false;
}