package org.firstinspires.ftc.teamcode.MechCat.ControlledPeriod.Tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@TeleOp(name = "ArmTest")
public class Test extends LinearOpMode {
    public ArmTEST arm;
        //arm.init();

    @Override
    public void runOpMode() {
        waitForStart();
        if (opModeIsActive()) {
            test();
        }

    }
    public void test() {
      //  arm.vTarget = 200;
    }
}
