package org.firstinspires.ftc.teamcode.MechCat.ControlledPeriod;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MechCat.ControlledPeriod.Arm;

@TeleOp(name = "ArmTest")
public class Test extends LinearOpMode {
    public Arm arm;
        //arm.init();

    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            test();
        }

    }
    public void test() {
        arm.vTarget = 200;
    }
}
