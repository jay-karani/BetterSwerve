package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SwerveModule {
    private MotorEx driveMotor;
    private CRServoEx rotationServo;
    private double podXOffset, podYOffset;

    public SwerveModule(HardwareMap hwMap, String motorName, boolean motorInverted,
                        String servoName, String analogName, double maxAnalogVolts, PIDFCoefficients servoPIDF, boolean servoReversed,
                        boolean analogReversed, double analogOffset, double xOffset, double yOffset) {

        this.driveMotor = new MotorEx(hwMap, motorName);
        driveMotor.setInverted(motorInverted).setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.rotationServo = new CRServoEx(hwMap, servoName, analogName, maxAnalogVolts, AngleUnit.RADIANS, CRServoEx.RunMode.OptimizedPositionalControl);
        rotationServo.setPIDF(servoPIDF).setInverted(SwerveConstants.flServoReversed);
        rotationServo.getAbsoluteEncoder().setReversed(analogReversed).zero(analogOffset);
        rotationServo.set(0);

        this.podXOffset = xOffset;
        this.podYOffset = yOffset;
    }

    public void setPower(double power){
        driveMotor.set(power);
    }
    public void rotateTo(double target){
        rotationServo.set(target);
    }
    public double getHeadingRad(){
        return rotationServo.getAbsoluteEncoder().getCurrentPosition();
    }

}
