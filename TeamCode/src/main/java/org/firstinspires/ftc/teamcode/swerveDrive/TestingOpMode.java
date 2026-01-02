package org.firstinspires.ftc.teamcode.swerveDrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@TeleOp
public class TestingOpMode extends OpMode {
    AbsoluteAnalogEncoder flAbsolute;
    SwerveModeServo flServo;
    Motor testMotor;
    SwerveDrivetrain swerve;

    @Override
    public void init() {
        //flAbsolute = new AbsoluteAnalogEncoder(hardwareMap, SwerveConstants.flAnalogName, SwerveConstants.ANALOG_VOLT_COMP, AngleUnit.RADIANS).setReversed(SwerveConstants.flAnalogReversed).zero(SwerveConstants.flAnalogOffset);
        //flServo = new SwerveModeServo(hardwareMap, SwerveConstants.flServoName, flAbsolute).setPIDF(SwerveConstants.flServoPIDF);
        //testMotor = new Motor(hardwareMap, SwerveConstants.blMotorName);

        swerve = new SwerveDrivetrain(hardwareMap);
    }

    @Override
    public void loop() {
        //testMotor.set(0.5);
        
        //flServo.set(Math.PI/2);
        //telemetry.addData("flPos", flServo.getAbsoluteEncoder().getCurrentPosition());
        //telemetry.update();
        swerve.startTeleopDrive();
        swerve.arcadeDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        swerve.updateConstants();
        if (gamepad1.aWasPressed()){
            SwerveConstants.lockFormation = !SwerveConstants.lockFormation;
        }
    }
}
