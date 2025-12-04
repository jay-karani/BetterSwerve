package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class SwerveConstants {

    public static final double MAX_X_VELOCITY = 3;
    public static final double MAX_Y_VELOCITY = 3;

    public static final String flMotorName = "fl_motor";
    public static final String flServoName = "fl_rotation";
    public static final String flAnalogName = "fl_absolute";
    public static final String blMotorName = "bl_motor";
    public static final String blServoName = "bl_rotation";
    public static final String blAnalogName = "bl_absolute";
    public static final String brMotorName = "br_motor";
    public static final String brServoName = "br_rotation";
    public static final String brAnalogName = "br_absolute";
    public static final String frMotorName = "fr_motor";
    public static final String frServoName = "fr_rotation";
    public static final String frAnalogName = "fr_absolute";

    public static PIDFCoefficients flServoPIDF = new PIDFCoefficients(0.01, 0, 0, 0);
    public static PIDFCoefficients blServoPIDF = new PIDFCoefficients(0.01, 0, 0, 0);
    public static PIDFCoefficients brServoPIDF = new PIDFCoefficients(0.01, 0, 0, 0);
    public static PIDFCoefficients frServoPIDF = new PIDFCoefficients(0.01, 0, 0, 0);
    public static final boolean flServoReversed = false;
    public static final boolean blServoReversed = false;
    public static final boolean brServoReversed = false;
    public static final boolean frServoReversed = false;

    public static final boolean flMotorReversed = false;
    public static final boolean blMotorReversed = false;
    public static final boolean brMotorReversed = false;
    public static final boolean frMotorReversed = false;

    public static final boolean flAnalogReversed = false;
    public static final boolean blAnalogReversed = false;
    public static final boolean brAnalogReversed = false;
    public static final boolean frAnalogReversed = false;

    public static double flAnalogOffsetRad = 6.0466;
    public static double blAnalogOffsetRad = 4.7116;
    public static double brAnalogOffsetRad = 1.9984;
    public static double frAnalogOffsetRad = 1.332;

    public static final double ANALOG_VOLT_COMP = 3.1865;

    public static final double GEAR_RATIO = 1/8.64;

    public static boolean field_centric = false;

    public static final double JOYSTICK_DEADBAND = 0.05;

    public static final double flPodXOffset = -1;
    public static final double flPodYOffset = 1;
    public static final double blPodXOffset = -1;
    public static final double blPodYOffset = -1;
    public static final double brPodXOffset = 1;
    public static final double brPodYOffset = -1;
    public static final double frPodXOffset = 1;
    public static final double frPodYOffset = 1;
}
