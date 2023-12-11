package org.firstinspires.ftc.teamcode.MechCat.ControlledPeriod;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
public class Arm extends OpMode
{

    public Arm() {

    }
    //HARDWARE
    private DcMotorEx LArmMotor, RArmMotor;

    //region PIDS
    public static PIDController vController;
    public static double Pv = 0.015, Iv = 0.002, Dv = 0.0005, Fv = 0.02; //
    // Pv = 0.029, Iv = 0.001, Dv = 0.0005, Fv = 0.03
    //
    public static int vTarget = 0;

    // PID tuning: static the above 3 variables, upload code, then tune

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LynxModule chub = hardwareMap.getAll(LynxModule.class).get(0);
        vController = new PIDController(Pv, Iv, Dv);

        LArmMotor = hardwareMap.get(DcMotorEx.class, "LArm");
        LArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RArmMotor = hardwareMap.get(DcMotorEx.class, "RArm");
        RArmMotor.setDirection(DcMotorEx.Direction.REVERSE);
        vTarget = 0;
    }

    @Override
    public void loop() {

        if (vTarget > 640)
            vTarget = 640;
        else if (vTarget < 30)
            vTarget = 30;

        //code
        int vPosition = LArmMotor.getCurrentPosition();
        vController.setPID(Pv, Iv, Dv);

        //region PID UPDATING
        double vPID = vController.calculate(vPosition, vTarget);

        double ticks_per_degree_tetrix = 3.84444444444444444444444444444;
        double vFeed = Math.cos(Math.toRadians((vTarget - 330) / ticks_per_degree_tetrix)) * Fv;

        LArmMotor.setPower(vPID + vFeed);
        RArmMotor.setPower(vPID + vFeed);
        //end code

        //region PID UPDATING
        LArmMotor.setPower(vPID + vFeed);
        RArmMotor.setPower(vPID + vFeed);
        //endregion PID UPDATING

        //region MANUAL MOVEMENT
        vTarget -= Math.round(5.0 * gamepad2.left_trigger);
        vTarget += Math.round(7.0 * gamepad2.right_trigger);

        telemetry.addData("vPosition", vPosition);
        telemetry.addData("vTarget", vTarget);
        telemetry.update();

        if (gamepad2.a) {
            vTarget = 200;
        }

        if (gamepad2.b) {
            vTarget = 400;
        }
    }
}
