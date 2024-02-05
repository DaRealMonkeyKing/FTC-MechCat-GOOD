package org.firstinspires.ftc.teamcode.MechCat.ControlledPeriod.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//@TeleOp
@Config
public class ArmTEST extends OpMode
{
    @Override
    public void init(){
        initialize();
    }
    public ArmTEST() {
        initialize();
    }
    //HARDWARE
    private DcMotorEx LArmMotor, RArmMotor, ExtendArmMotor;

    //region PIDS
    public static PIDController armController, slideController;
    public static double armPv = 0.015, armIv = 0.002, armDv = 0.0005, armFv = 0.02,
                        slidePv = 0.015, slideIv = 0.002, slideDv = 0.0005, slideFv = 0.02;
    public static int armvTarget, slidevTarget = 0;

    // PID tuning: static the above 3 variables, upload code, then tune

    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LynxModule chub = hardwareMap.getAll(LynxModule.class).get(0);
        armController = new PIDController(armPv, armIv, armDv);

        LArmMotor = hardwareMap.get(DcMotorEx.class, "LArm");
        LArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RArmMotor = hardwareMap.get(DcMotorEx.class, "perpendicularEncoder");
        RArmMotor.setDirection(DcMotorEx.Direction.REVERSE);
        ExtendArmMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        ExtendArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtendArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {
        armPID();
    }

    public void armPID(){
        //code
        int vPosition = LArmMotor.getCurrentPosition();
        armController.setPID(armPv, armIv, armDv);

        //region PID UPDATING
        double vPID = armController.calculate(vPosition, armvTarget);

        double ticks_per_degree_tetrix = 3.84444444444444444444444444444;
        double vFeed = Math.cos(Math.toRadians((armvTarget - 0) / ticks_per_degree_tetrix)) * armFv;

        LArmMotor.setPower(vPID + vFeed);
        RArmMotor.setPower(vPID + vFeed);
        //end code

        //region PID UPDATING
        LArmMotor.setPower(vPID + vFeed);
        RArmMotor.setPower(vPID + vFeed);
        //endregion PID UPDATING

        telemetry.addData("vPosition", vPosition);
        telemetry.addData("vTarget", armvTarget);
    }

    public void ArmTo(int Target) {
        //region MANUAL MOVEMENT
        armvTarget = Target;
    }

    public void SlideTo(int Target) {
        //code
        int vPosition = ExtendArmMotor.getCurrentPosition();
        slideController.setPID(armPv, armIv, armDv);

        //region PID UPDATING
        double vPID = slideController.calculate(vPosition, slidevTarget);

        double ticks_per_degree_tetrix = 3.84444444444444444444444444444;
        double vFeed = Math.cos(Math.toRadians((slidevTarget - 0) / ticks_per_degree_tetrix)) * armFv;

        ExtendArmMotor.setPower(vPID + vFeed);
        //end code

        //region PID UPDATING
        ExtendArmMotor.setPower(vPID + vFeed);
        //endregion PID UPDATING

        //region MANUAL MOVEMENT
        slidevTarget = Target;

        telemetry.addData("vPosition", vPosition);
        telemetry.addData("vTarget", slidevTarget);
    }
}
