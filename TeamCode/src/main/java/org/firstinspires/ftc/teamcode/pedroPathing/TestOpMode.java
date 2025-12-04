package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestOpMode extends OpMode {
    SwerveModule flMod;

    @Override
    public void init() {
        flMod = new SwerveModule(hardwareMap, SwerveConstants.flMotorName, SwerveConstants.flMotorReversed, SwerveConstants.flServoName,
                SwerveConstants.flAnalogName, SwerveConstants.ANALOG_VOLT_COMP, SwerveConstants.flServoPIDF, SwerveConstants.flServoReversed,
                SwerveConstants.flAnalogReversed, SwerveConstants.flAnalogOffsetRad,SwerveConstants.flPodXOffset, SwerveConstants.flPodYOffset);
    }

    @Override
    public void loop() {
        flMod.rotateTo(0);
        flMod.setPower(-gamepad1.left_stick_y);
        telemetry.addData("flPos", flMod.getHeadingRad());
        telemetry.update();
    }
}
