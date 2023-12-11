package org.firstinspires.ftc.teamcode.MechCat.ControlledPeriod;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.Claw1ClosePos;
import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.Claw2ClosePos;
import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.Claw1OpenPos;
import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.Claw2OpenPos;
import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.ClawServoBoard;
import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.ClawServoGround;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * This opmode demonstrates how one would implement field centric control using
 * `SampleMecanumDrive.java`. This file is essentially just `TeleOpDrive.java` with the addition of
 * field centric control. To achieve field centric control, the only modification one needs is to
 * rotate the input vector by the current heading before passing it into the inverse kinematics.
 * <p>
 * See lines 42-57.
 */
@TeleOp(group = "advanced")
@Config
public class FieldCentricDrive extends LinearOpMode {
    //region PIDS
    public static PIDController vController; // static when tuning
    public static double Pv = 0.015, Iv = 0.002, Dv = 0.0005, Fv = 0.02; // Pv = 0.003, Iv = 0, Dv = 0, Fv = 0.3
    // Pv = 0.029, Iv = 0, Dv = 0.0013, Fv = 0.03 // static when tuning
    // Dv = 0.0005, Fv = 0.03, Iv = 0.001, Pv = 0.029
    public int vTarget = 20; // static when tuning
    public static boolean lastStateClaw = false;

    DcMotor LArmMotor; // expansion hub: 3
    DcMotor RArmMotor; // expansion hub: 2
    Servo planeServo, claw1, claw2, clawServo; // claw1: 2 Control Hub claw2: 4 control hub
        // clawServo: 0 Expansion hub

    @Override
    public void runOpMode() {
        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);


        // PID (make it look nicer later we just need it working hahahahahahahahaah)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LynxModule chub = hardwareMap.getAll(LynxModule.class).get(0);
        vController = new PIDController(Pv, Iv, Dv);

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        LArmMotor = hardwareMap.get(DcMotorEx.class, "LArm");
        LArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RArmMotor = hardwareMap.get(DcMotorEx.class, "RArm");
        RArmMotor.setDirection(DcMotorEx.Direction.REVERSE);
        vTarget = 20;

        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        planeServo = hardwareMap.get(Servo.class, "planeServo");

        claw1.setPosition(Claw1ClosePos);
        claw2.setPosition(Claw2ClosePos);
        clawServo.setPosition(ClawServoGround);
        planeServo.setPosition(0);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            arm(gamepad1.right_trigger, gamepad1.left_trigger); // param: up, down
            move(drive, gamepad1.right_bumper);
            OCclaw(gamepad1.a);
            shootPlane(gamepad1.y);
            telemetry.update();
        }
    }
    private void move(SampleMecanumDrive drive, boolean crawl){
        // Toggle crawl
        double mltp = 1;
        if (crawl)
            mltp = 0.5;
        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Create a vector from the gamepad dpad inputs
        double y1 = gamepad1.dpad_up ? 1 : 0;
        double y2 = gamepad1.dpad_down? -1 : 0;
        double y = y1 + y2;
        double x1 = gamepad1.dpad_right ? 1 : 0;
        double x2 = gamepad1.dpad_left ? -1 : 0;
        double x = x1 + x2;
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                y,
                -x
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        (input.getX() - gamepad1.left_stick_y) * mltp,
                        (input.getY() - gamepad1.left_stick_x) * mltp,
                        -gamepad1.right_stick_x * mltp
                )
        );

        // Update everything. Odometry. Etc.
        drive.update();

        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
    }
    private void arm(double upControl, double downControl){
        //code
        int vPosition = LArmMotor.getCurrentPosition();
        vController.setPID(Pv, Iv, Dv);

        // set limit for arm movement
        if (vTarget > 640)
            vTarget = 640;
        else if (vTarget < 20)
            vTarget = 20;

        if (vTarget > 250) {
            double pos = (435 - ((vTarget - 100) / (11/3f))) / 300;
            if (pos > 1)
                pos = 1f;
            clawServo.setPosition(pos);
        } else {
            clawServo.setPosition(ClawServoGround);
        }

        //region PID UPDATING
        double vPID = vController.calculate(vPosition, vTarget);

        double ticks_per_degree_tetrix = 3.84444444444444444444444444444;
        double vFeed = Math.cos(Math.toRadians((vTarget - 300) / ticks_per_degree_tetrix)) * Fv;

        LArmMotor.setPower(vPID + vFeed);
        RArmMotor.setPower(vPID + vFeed);
        //end code

        //region PID UPDATING
        LArmMotor.setPower(vPID + vFeed);
        RArmMotor.setPower(vPID + vFeed);
        //endregion PID UPDATING

        //region MANUAL MOVEMENT
        vTarget -= Math.round(10.0 * downControl); // 5
        vTarget += Math.round(10.0 * upControl); // 5

        telemetry.addData("vPosition", vPosition);
        telemetry.addData("vTarget", vTarget);
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
        telemetry.addData("Claw open?: ", claw1.getPosition());
    }

    public void shootPlane(boolean power) {
        if (power)
            planeServo.setPosition(0.5);
        else
            planeServo.setPosition(0);
        telemetry.addData("Servo: ", power);
    }
}