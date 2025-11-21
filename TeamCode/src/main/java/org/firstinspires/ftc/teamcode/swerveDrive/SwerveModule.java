package org.firstinspires.ftc.teamcode.swerveDrive;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.opmodecontrol.ActiveOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class SwerveModule{

    CRServoEx rotationServo;
    MotorEx driveMotor;
    public static PIDFCoefficients pidf = new PIDFCoefficients(0.01, 0, 0, 0);
    public static double voltageCompensation = 3.1865;
    AngleUnit radians = AngleUnit.RADIANS;

    public SwerveModule(HardwareMap hwMap, String servoName, String analogName, double voltageCompensation, boolean servoReversed, double servoOffset,
                        String motorName, boolean motorReversed){
        this.rotationServo = new CRServoEx(hwMap, servoName, analogName, voltageCompensation, radians, CRServoEx.RunMode.OptimizedPositionalControl);
        rotationServo.setInverted(servoReversed);
        rotationServo.getAbsoluteEncoder().zero(servoOffset);
        rotationServo.setPIDF(pidf).set(0);

        this.driveMotor = new MotorEx(hwMap, motorName, Motor.GoBILDA.BARE);
        driveMotor.setInverted(motorReversed).setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public double getPodHeading(){
        return rotationServo.getCurrentPosition();
    }
}
