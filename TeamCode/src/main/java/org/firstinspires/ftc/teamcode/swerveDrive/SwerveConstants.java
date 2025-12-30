package org.firstinspires.ftc.teamcode.swerveDrive;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@Configurable
public class SwerveConstants {
    public static boolean useFieldCentric = false;
    public static boolean overrideDrive = false;

    public static double initialHeading = 0;
    public static Motor.ZeroPowerBehavior brakeModeTele = Motor.ZeroPowerBehavior.BRAKE;

    public static double MAX_X_VELOCITY = 3;
    public static double MAX_Y_VELOCITY = 3;

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

    public static PIDFCoefficients flServoPIDF = new PIDFCoefficients(0.3, 0, 0, 0);
    public static PIDFCoefficients blServoPIDF = new PIDFCoefficients(0.3, 0, 0, 0);
    public static PIDFCoefficients brServoPIDF = new PIDFCoefficients(0.3, 0, 0, 0);
    public static PIDFCoefficients frServoPIDF = new PIDFCoefficients(0.3, 0, 0, 0);

    public static boolean flMotorReversed = true;
    public static boolean blMotorReversed = false;
    public static boolean brMotorReversed = false;
    public static boolean frMotorReversed = true;

    public static boolean flAnalogReversed = false;
    public static boolean blAnalogReversed = false;
    public static boolean brAnalogReversed = false;
    public static boolean frAnalogReversed = false;

    public static double flAnalogOffset = 6.0466;
    public static double blAnalogOffset = 4.7116;
    public static double brAnalogOffset = 1.9984;
    public static double frAnalogOffset = 1.332;

    public static double ANALOG_VOLT_COMP = 3.1865;

    public static final double GEAR_RATIO = 1/8.64;

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
