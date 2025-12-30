package org.firstinspires.ftc.teamcode.swerveDrive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SwerveModule {
    private Motor driveMotor;
    private SwerveModeServo rotateServo;
    private double podXLocation;
    private double podYLocation;

    /*
        Constructor
     */
    public SwerveModule(HardwareMap hwMap, String motorName, boolean motorReversed, String servoName, PIDFCoefficients pidfCoeffs, double podX, double podY,
                        String analogName, double analogRange, boolean analogReversed, double analogOffset){

        this.driveMotor = new Motor(hwMap, motorName).setInverted(motorReversed);
        this.driveMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        this.rotateServo = new SwerveModeServo(hwMap, servoName, analogName, analogRange, AngleUnit.RADIANS);
        this.rotateServo.getAbsoluteEncoder().setReversed(analogReversed).zero(analogOffset);
        this.rotateServo.setPIDF(pidfCoeffs);
        this.podXLocation = podX;
        this.podYLocation = podY;
    }

    /*
        Hardware Write Methods
     */
    public void setDrivePower(double power){
        driveMotor.set(power);
    }
    public void rotateTo(double target){
        rotateServo.set(target);
    }
    public double getPodHeading(){
        return rotateServo.getAbsoluteEncoder().getCurrentPosition();
    }
    public void setBrakemode(Motor.ZeroPowerBehavior brakemode){
        driveMotor.setZeroPowerBehavior(brakemode);
    }
    public double getPodX(){
        return podXLocation;
    }
    public double getPodY(){
        return podYLocation;
    }

    /*
        Helper Methods
     */
    public Motor getMotor(){
        return driveMotor;
    }
    public SwerveModeServo getServo(){
        return rotateServo;
    }
    public void setMotor(Motor motor){
        driveMotor = motor;
    }
    public void setServo(SwerveModeServo servo){
        rotateServo = servo;
    }
    public void updateConstants(boolean motorReversed, boolean analogReversed,
                                double analogZero, PIDFCoefficients pidfCoeffs){
        driveMotor.setInverted(motorReversed);
        rotateServo.setPIDF(pidfCoeffs).getAbsoluteEncoder().zero(analogZero).setReversed(analogReversed);
    }
    public void updatePIDF(PIDFCoefficients pidf){
        rotateServo.setPIDF(pidf);
    }
}
