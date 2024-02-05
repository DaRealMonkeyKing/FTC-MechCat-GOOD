package org.firstinspires.ftc.teamcode.MechCat.ControlledPeriod;

import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.Claw1ClosePos;
import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.Claw1OpenPos;
import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.Claw2ClosePos;
import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.Claw2OpenPos;
import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.ClawServoBoard;
import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.ClawServoGround;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.Math;

@Config
@TeleOp(name = "TeleOp")
public class NewBotController extends LinearOpMode {
    DcMotor RFMotor, RBMotor, LFMotor, LBMotor;
    DcMotorEx LArmMotor, RArmMotor, ExtendArmMotor;
    Servo planeServo, claw1, claw2, clawServo;
    ArmController arm;
    Thread PID;
    public double armPos = 0, slidePos = -800;
    public boolean lastStateClaw = false, lastStateOut = false, outtake = false;

    @Override
    public void runOpMode() {
        // connecting variable names with device names set on the Android Phone
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        LArmMotor = hardwareMap.get(DcMotorEx.class, "perpendicularEncoder");
        RArmMotor = hardwareMap.get(DcMotorEx.class, "RArm");
        ExtendArmMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        planeServo = hardwareMap.get(Servo.class, "planeServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        claw1.setPosition(Claw1ClosePos);
        claw2.setPosition(Claw2ClosePos);
        clawServo.setPosition(ClawServoGround);
        planeServo.setPosition(0.5);

        ExtendArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ExtendArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LynxModule chub = hardwareMap.getAll(LynxModule.class).get(0);

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = new ArmController(LArmMotor, RArmMotor, ExtendArmMotor);
        arm.PIDactive(true);
        PID = new Thread(arm);
        PID.start();

        // Send information to Control Hub
        telemetry.addData("Initialization", " is a success");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            run();
        }
        arm.PIDactive(false);
        PID.stop();
    }

    public void run() {
        move(gamepad1.right_bumper);
//        moveArm(gamepad1.right_trigger, gamepad1.left_trigger);
//        extendArm(gamepad1.y, gamepad1.a);
        outtake(gamepad1.a);
        OCclaw(gamepad1.x);
        shootPlane(gamepad1.b);
        telemetry.update();
    }

    public void move(boolean crawl) {
        double mltp = 0.8;

        if (crawl)
            mltp = 0.4;

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

    public void outtake(boolean activate) {
        if (activate && !lastStateOut) {
            if (outtake == false) {
                arm.ArmTo(1073);
                slidePos = -800;
                outtake = true;
            }else{
                arm.ArmTo(0);
                arm.SlideTo(0);
                slidePos = -800;
                outtake = false;
            }
        }
        lastStateOut = activate;
        if (outtake){
            extendArm(gamepad1.y, gamepad1.a);
            clawServo.setPosition(ClawServoBoard);
        }
    }

    public void moveArm(double up, double down){
        double power = (up - down)/10;

        armPos = armPos + power;

        int fixedArmV = (int) Math.round(armPos);

        if (fixedArmV < 0) {
            fixedArmV = 0;
            armPos = 0;
        }

        arm.ArmTo(fixedArmV);

        telemetry.addData("Arm Position", arm.armPosition);
        telemetry.addData("Arm Target", arm.armTarget);
    }

    public void extendArm(boolean up, boolean down){
        double extend = up? 0 : 1;
        double retract = down? 0 : 1;
        double power = (extend - retract);

        slidePos = slidePos + power;

        int fixedArmV = (int) Math.round(slidePos);

        if (fixedArmV > -800){
            fixedArmV = -800;
            slidePos = -800;
        } else if (fixedArmV < -2200) { // for some reason the slider extends to NEGATIVE (fixed in other methods)
            fixedArmV = -2200;
            slidePos = -2200;
        }

        arm.SlideTo(-fixedArmV);

        telemetry.addData("Slide Position", arm.slidePosition * -1);
        telemetry.addData("Slide Target", arm.slideTarget * -1);
    }

    private void OCclaw(boolean down) {
        if (down && !lastStateClaw) {
            if (Math.abs(claw1.getPosition() - Claw1OpenPos) < 0.01) {
                claw1.setPosition(Claw1ClosePos);
                claw2.setPosition(Claw2ClosePos);
            }else {
                claw1.setPosition(Claw1OpenPos);
                claw2.setPosition(Claw2OpenPos);
            }
        }
        lastStateClaw = down;
        if (Math.abs(claw1.getPosition() - Claw1OpenPos) < 0.01){
            clawServo.setPosition(ClawServoGround);
        } else {
            clawServo.setPosition(ClawServoBoard / 3);
        }

        telemetry.addData("Claw open?: ", claw1.getPosition());
    }

    public void shootPlane(boolean power) {
        if (power)
            planeServo.setPosition(0);
        else
            planeServo.setPosition(0.5);
        telemetry.addData("Servo: ", power);
    }
}
