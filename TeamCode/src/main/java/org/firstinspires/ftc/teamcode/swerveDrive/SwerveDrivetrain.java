package org.firstinspires.ftc.teamcode.swerveDrive;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class SwerveDrivetrain extends CustomDrivetrain{

    private VoltageSensor voltageSensor;
    public SwerveModule flModule, blModule, brModule, frModule;
    public SwerveModule[] swerveModules;
    double[] wheelSpeeds, targetHeadings, currentHeadings, angleErrors, cachedAngles;

    public SwerveDrivetrain(HardwareMap hwMap){
        flModule = new SwerveModule(hwMap, SwerveConstants.flMotorName, SwerveConstants.flMotorReversed,
                SwerveConstants.flServoName, SwerveConstants.flServoPIDF, SwerveConstants.flPodXOffset, SwerveConstants.flPodYOffset,
                SwerveConstants.flAnalogName, SwerveConstants.ANALOG_VOLT_COMP, SwerveConstants.flAnalogReversed, SwerveConstants.flAnalogOffset);

        blModule = new SwerveModule(hwMap, SwerveConstants.blMotorName, SwerveConstants.blMotorReversed,
                SwerveConstants.blServoName, SwerveConstants.blServoPIDF, SwerveConstants.blPodXOffset, SwerveConstants.blPodYOffset,
                SwerveConstants.blAnalogName, SwerveConstants.ANALOG_VOLT_COMP, SwerveConstants.blAnalogReversed, SwerveConstants.blAnalogOffset);

        brModule = new SwerveModule(hwMap, SwerveConstants.brMotorName, SwerveConstants.brMotorReversed,
                SwerveConstants.brServoName, SwerveConstants.brServoPIDF, SwerveConstants.brPodXOffset, SwerveConstants.brPodYOffset,
                SwerveConstants.brAnalogName, SwerveConstants.ANALOG_VOLT_COMP, SwerveConstants.brAnalogReversed, SwerveConstants.brAnalogOffset);

        frModule = new SwerveModule(hwMap, SwerveConstants.frMotorName, SwerveConstants.frMotorReversed,
                SwerveConstants.frServoName, SwerveConstants.frServoPIDF, SwerveConstants.frPodXOffset, SwerveConstants.frPodYOffset,
                SwerveConstants.frAnalogName, SwerveConstants.ANALOG_VOLT_COMP, SwerveConstants.frAnalogReversed, SwerveConstants.frAnalogOffset);

        swerveModules = new SwerveModule[]{flModule, blModule, brModule, frModule};

        wheelSpeeds = new double[swerveModules.length];
        targetHeadings = new double[swerveModules.length];
        currentHeadings = new double[swerveModules.length];
        angleErrors = new double[swerveModules.length];
        cachedAngles = new double[swerveModules.length];

        voltageSensor = hwMap.get(VoltageSensor.class, "Control Hub");
    }

    @Override
    public void arcadeDrive(double forward, double strafe, double rotation) {
        double translateY = -forward;
        double strafeX = -strafe;
        double rotate = rotation;

        boolean joystickIsIdle = ((Math.abs(translateY) < SwerveConstants.JOYSTICK_DEADBAND) &&
                (Math.abs(strafeX) < SwerveConstants.JOYSTICK_DEADBAND) && (Math.abs(rotation) < SwerveConstants.JOYSTICK_DEADBAND));

        for (int i = 0; i < swerveModules.length; i++){
            Vector translateVector = new Vector();
            translateVector.setOrthogonalComponents(0, translateY);

            Vector strafeVector = new Vector();
            strafeVector.setOrthogonalComponents(strafeX, 0);

            Vector rotateVector = new Vector();
            rotateVector.setOrthogonalComponents(swerveModules[i].getPodX(), swerveModules[i].getPodY());
            rotateVector.rotateVector(Math.PI/2);
            rotateVector.setMagnitude(rotate);

            Vector resultantVector = translateVector.plus(strafeVector).plus(rotateVector);

            wheelSpeeds[i] = resultantVector.getMagnitude();
            targetHeadings[i] = resultantVector.getTheta();
            currentHeadings[i] = swerveModules[i].getPodHeading();

            double tempDiff = Math.abs(targetHeadings[i] - currentHeadings[i]);
            angleErrors[i] = Math.min(tempDiff, 2*Math.PI - tempDiff);

        }

        double maxRequiredWheelPower = Math.max(Math.max(wheelSpeeds[0], wheelSpeeds[1]),
                Math.max(wheelSpeeds[2], wheelSpeeds[3]));
        if (maxRequiredWheelPower > 1){
            for (int i = 0; i < swerveModules.length; i++){wheelSpeeds[i] /= maxRequiredWheelPower;}
        }

        for (int i = 0; i < swerveModules.length; i++){
            if (angleErrors[i] > (Math.PI/2)){
                targetHeadings[i] = (targetHeadings[i] + Math.PI) % (2*Math.PI);
                wheelSpeeds[i] *= -1;
                double tempDiff = Math.abs(targetHeadings[i] - currentHeadings[i]);
                angleErrors[i] = Math.min(tempDiff, 2*Math.PI - tempDiff);
            }
            wheelSpeeds[i] *= Math.cos(angleErrors[i]);

            if (!joystickIsIdle){
                cachedAngles[i] = targetHeadings[i];
            }
        }

        for (int i = 0; i < swerveModules.length; i++) {
            if (!SwerveConstants.lockFormation){
                double commandedAngle = joystickIsIdle ? cachedAngles[i] : targetHeadings[i];
                swerveModules[i].rotateTo(commandedAngle);
                swerveModules[i].setDrivePower(wheelSpeeds[i]);
            } else{
                Vector lockVector = new Vector();
                lockVector.setOrthogonalComponents(swerveModules[i].getPodX(), swerveModules[i].getPodY());
                swerveModules[i].rotateTo(lockVector.getTheta());
                swerveModules[i].setDrivePower(0);
            }
        }
    }

    @Override
    public void updateConstants() {
        flModule.updateConstants(SwerveConstants.flMotorReversed, SwerveConstants.flAnalogReversed,
                SwerveConstants.flAnalogOffset, SwerveConstants.flServoPIDF);

        blModule.updateConstants(SwerveConstants.blMotorReversed, SwerveConstants.blAnalogReversed,
                SwerveConstants.blAnalogOffset, SwerveConstants.blServoPIDF);

        brModule.updateConstants(SwerveConstants.brMotorReversed, SwerveConstants.brAnalogReversed,
                SwerveConstants.brAnalogOffset, SwerveConstants.brServoPIDF);

        frModule.updateConstants(SwerveConstants.frMotorReversed, SwerveConstants.frAnalogReversed,
                SwerveConstants.frAnalogOffset, SwerveConstants.frServoPIDF);
    }

    @Override
    public void breakFollowing() {
        for (SwerveModule module : swerveModules){
            module.setDrivePower(0);
            module.setBrakemode(Motor.ZeroPowerBehavior.FLOAT);
        }
    }

    @Override
    public void startTeleopDrive() {
        for (SwerveModule module : swerveModules){
            module.setBrakemode(SwerveConstants.brakeModeTele);
        }
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        for (SwerveModule module : swerveModules){
            if (brakeMode){
                module.setBrakemode(Motor.ZeroPowerBehavior.BRAKE);
            }
            else {
                module.setBrakemode(Motor.ZeroPowerBehavior.FLOAT);
            }
        }
    }

    @Override
    public double xVelocity() {
        return SwerveConstants.MAX_X_VELOCITY;
    }

    @Override
    public double yVelocity() {
        return SwerveConstants.MAX_Y_VELOCITY;
    }

    @Override
    public void setXVelocity(double xMovement) {
        SwerveConstants.MAX_X_VELOCITY = xMovement;
    }

    @Override
    public void setYVelocity(double yMovement) {
        SwerveConstants.MAX_Y_VELOCITY = yMovement;
    }

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    @Override
    public String debugString() {
        return "FLCurrentPos: " + flModule.getPodHeading() +
                "\n BLCurrentPos: " + blModule.getPodHeading() +
                "\n BRCurrentPos: " + brModule.getPodHeading() +
                "\n FRCurrentPos: " + frModule.getPodHeading() +
                "\n FLTargetPos: " + targetHeadings[0] +
                "\n BLTargetPos: " + targetHeadings[1] +
                "\n BRTargetPos: " + targetHeadings[2] +
                "\n FRTargetPos: " + targetHeadings[3];
    }
}
