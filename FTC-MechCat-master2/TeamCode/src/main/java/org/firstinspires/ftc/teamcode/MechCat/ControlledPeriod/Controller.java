package org.firstinspires.ftc.teamcode.MechCat.ControlledPeriod;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Controller")
@Config
public class Controller extends OpMode {

    /*
     * R = RIGHT
     * L = LEFT
     * B = BACK
     * F = FRONT
     */
    // HARDWARE
    DcMotor RFMotor, RBMotor, LFMotor, LBMotor;
    DcMotor LArmMotor; // expansion hub: 3
    DcMotor RArmMotor; // expansion hub: 2
    Servo planeServo, claw, clawServo;

    //region PIDS
    public static PIDController vController; // static when tuning
    public static double Pv = 0.029, Iv = 0.001, Dv = 0.0005, Fv = 0.03; // Pv = 0.003, Iv = 0, Dv = 0, Fv = 0.3
    // Pv = 0.029, Iv = 0, Dv = 0.0013, Fv = 0.03 // static when tuning
    // Dv = 0.0005, Fv = 0.03, Iv = 0.001, Pv = 0.029
    public int vTarget = 0; // static when tuning

    @Override
    public void init() {
        // connecting variable names with device names set on the Android Phone
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        planeServo = hardwareMap.get(Servo.class, "Launcher");
        claw = hardwareMap.get(Servo.class, "claw");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        planeServo.setDirection(Servo.Direction.REVERSE);

        //set servos
        claw.setPosition(1.00 / 6);
        planeServo.setPosition(0.5);
        clawServo.setPosition(0.15);

        // PID (make it look nicer later we just need it working hahahahahahahahaah)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LynxModule chub = hardwareMap.getAll(LynxModule.class).get(0);
        vController = new PIDController(Pv, Iv, Dv);

        LArmMotor = hardwareMap.get(DcMotorEx.class, "LArm");
        LArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RArmMotor = hardwareMap.get(DcMotorEx.class, "RArm");
        RArmMotor.setDirection(DcMotorEx.Direction.REVERSE);
        vTarget = 20;

        // Send information to Control Hub
        telemetry.addData("Initialization", " is a success");
        telemetry.update();
    }

    @Override
    public void loop() {
        move(gamepad1.right_bumper); // param: crawl
        moveArm(gamepad1.right_trigger, gamepad1.left_trigger); // param: up, down
        shootPlane(gamepad1.y); // param: button
        OPClaw(gamepad1.a); // param: button
    }

    public void move(boolean crawl) {
        //double INVERT_RIGHT_MOTORS = -1;
        double mltp = 0.8;
        if (crawl)
            mltp = 0.4;

        double INVERT_BACK_MOTOR_DRIVE = -1;
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;


        double RFMotorPower = (drive + strafe + turn);
        double LFMotorPower = (drive - strafe - turn);
        double RBMotorPower = (drive - strafe + turn);
        double LBMotorPower = (drive + strafe - turn);

        // Power the motors!
        RFMotor.setPower(RFMotorPower * mltp);
        LFMotor.setPower(LFMotorPower * mltp);
        RBMotor.setPower(RBMotorPower * mltp);
        LBMotor.setPower(LBMotorPower * mltp);

        // Send power data to the Driver Station
        telemetry.addData("RF Target Power", RFMotorPower);
        telemetry.addData("LF Target Power", LFMotorPower);
        telemetry.addData("RB Target Power", RBMotorPower);
        telemetry.addData("LB Target Power", LBMotorPower);
    }

    public void shootPlane(boolean power) {
        if (power)
            planeServo.setPosition(0);
        else
            planeServo.setPosition(0.5);
        telemetry.addData("Servo: ", power);
    }

    public void moveArm(double upControl, double downControl) {
        //code
        int vPosition = LArmMotor.getCurrentPosition();
        vController.setPID(Pv, Iv, Dv);

        // set limit for arm movement
        if (vTarget > 640)
            vTarget = 640;
        else if (vTarget < 10)
            vTarget = 10;

        if (vTarget > 320) {
            clawServo.setPosition(0.65);
        } else {
            clawServo.setPosition(0.15);
        }

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
        vTarget -= Math.round(5.0 * downControl);
        vTarget += Math.round(5.0 * upControl);

        telemetry.addData("vPosition", vPosition);
        telemetry.addData("vTarget", vTarget);
        telemetry.update();
    }

    // open / close claw
    public static boolean lastStateClaw;
    public void OPClaw(boolean down) {
        double pos;
        if (down && !lastStateClaw) {
            if (claw.getPosition() == 0)
                pos = 1.00 / 6;
            else
                pos = 0;
            claw.setPosition(pos);
        }
        lastStateClaw = down;
        telemetry.addData("Claw open?: ", claw.getPosition());
    }
}
