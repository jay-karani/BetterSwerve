package org.firstinspires.ftc.teamcode.swerveDrive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TestClass extends CommandOpMode {

    SwerveModule module;
    @Override
    public void initialize() {
         module = new SwerveModule(hardwareMap, "fl_rotation", "fl_absolute", 3.1865, true, 0, "fl_motor", false);

    }

    @Override
    public void run(){
        while (opModeIsActive() && !isStopRequested()){


            telemetry.addData("heading", module.getPodHeading());
            telemetry.update();
        }
    }
}
