/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.NewOld;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;

import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.Claw1ClosePos;
import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.Claw2ClosePos;
import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.Claw1OpenPos;
import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.Claw2OpenPos;
import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.ClawServoBoard;
import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.ClawServoGround;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */

// field 142 x 142 inches (6 tiles x 6 tiles)
// tile 24 x 24 inches
// 3rd tile from pixel drop area are bars
// 8 inches from wall (backboard)
// 12 inches from backboard to blue tape
// 0.5 inch for half of metal bar

// 8 inches to center from back
// 7 inches to center from front
// 6.5 inches to center from sides

// claw able to release independently
// change dpad up to extend slider
// dpad down to retract slider
// dpad left and right to release corresponding claw

public class Autonomous extends LinearOpMode {

    protected static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    protected static final String TFOD_MODEL_ASSET = "OurCoolModel2.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    //private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    protected static final String[] LABELS = {
            "B_Prop",
            "R_Prop",
    };

    public String side = "idk";

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    protected TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    protected VisionPortal visionPortal;
    protected Servo claw1, claw2, clawServo;

    protected DcMotorEx LArmMotor, RArmMotor;

    //region PIDS
    protected static PIDController vController;
    protected static double Pv = 0.015, Iv = 0.002, Dv = 0.0005, Fv = 0.02; //
    // Pv = 0.029, Iv = 0.001, Dv = 0.0005, Fv = 0.03
    //
    protected static int vTarget = 0;

    protected double spdLMT = 14;

    protected SampleMecanumDrive drive;

    protected double x, y;


    @Override

    public void runOpMode() {
        // sample mecanum drive for the trajectory
        initialize();
        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {
            // run redside roadrunner
            RRRoadRunner(drive);
        }
        // update pose
        PoseStorage.currentPose = drive.getPoseEstimate();

    }   // end runOpMode()

    public void RRRoadRunner(SampleMecanumDrive drive){
        //nothing
    }

    public void weGoRam(SampleMecanumDrive drive, Trajectory traj){
        int localSpdLMT = 14;
        Trajectory forward = drive.trajectoryBuilder(traj.end())
                .forward(24,
                        SampleMecanumDrive.getVelocityConstraint(localSpdLMT, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory backwards = drive.trajectoryBuilder(forward.end())
                .back(11,
                        SampleMecanumDrive.getVelocityConstraint(localSpdLMT, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        clawServo.setPosition(ClawServoGround);
        drive.followTrajectory(forward);
        sleep(500);
        drive.followTrajectory(backwards);
        sleep(500);
    }
    // TODO: Make this method
    public void placePixelLine(){
        // place pixel onto line
        clawServo.setPosition(ClawServoGround);
        sleep(1000);
        claw1.setPosition(Claw1OpenPos); // open
        sleep(500);
        clawServo.setPosition(ClawServoBoard);
    }

    // TODO: Make this method
    //HARDWARE
    public void placePixelBoard(int target){

        int vPosition = LArmMotor.getCurrentPosition();

        if (Math.abs(vPosition - target) <= 10) {
            vTarget = target;
            double pos = (425 - ((vTarget - 100) / (11/3f))) / 300;
            if (pos > 1)
                pos = 1f;
            clawServo.setPosition(pos);
            claw2.setPosition(Claw2OpenPos);
            claw1.setPosition(Claw1OpenPos);
        }
        else if (vPosition > target){
            vTarget -= 5;
        } else if (vPosition < target) {
            vTarget += 5;
        }

        if (vTarget > 750)
            vTarget = 750;
        else if (vTarget < 20)
            vTarget = 20;

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

        telemetry.addData("vPosition", vPosition);
        telemetry.addData("vTarget", vTarget);
        telemetry.update();
    }

    public void placeBoard(){
        // TODO: Place pixel on board
        while (claw2.getPosition() == Claw2ClosePos) {
            placePixelBoard(680);
            if (!opModeIsActive()) break;
        }

        clawServo.setPosition(ClawServoBoard);

        while (vTarget > 30) {
            placePixelBoard(20);
            if (!opModeIsActive()) break;
        }
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */

    public void initialize() {
        // sample mecanum drive for the trajectory
        drive = new SampleMecanumDrive(hardwareMap);
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        // init claw pos
        clawServo.setPosition(ClawServoBoard);
        claw1.setPosition(Claw1ClosePos);
        claw2.setPosition(Claw2ClosePos);
        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        LynxModule chub = hardwareMap.getAll(LynxModule.class).get(0);
        vController = new PIDController(Pv, Iv, Dv);

        LArmMotor = hardwareMap.get(DcMotorEx.class, "LArm");
        LArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RArmMotor = hardwareMap.get(DcMotorEx.class, "RArm");
        RArmMotor.setDirection(DcMotorEx.Direction.REVERSE);
        vTarget = 0;
    }

    public void getSide() {
        boolean loop = true;
        int count = 3000; //
        while (loop) {
            if (isStopRequested()) loop = false;
            //Init the model whatever
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            double biggestArea = 0;

            // Step through the list of recognitions and check info for each one.
            for (Recognition recognition : currentRecognitions) {
                x = (recognition.getLeft() + recognition.getRight()) / 2;
                y = (recognition.getTop() + recognition.getBottom()) / 2;
                double width = (recognition.getRight() - recognition.getLeft());
                double height = (recognition.getBottom() - recognition.getTop());
                String label = recognition.getLabel();

                if (width * height > biggestArea){
                    side = String.valueOf(label.charAt(0));
                    biggestArea = width * height;
                    if (x < 213)
                        side += "L";
                    else if (x > 430)
                        side += "R";
                    else if (213 < x && x < 460)
                        side += "M";
                    else
                        side += "N"; // if N means broken
                    telemetry.addData("We going: ", side);
                    telemetry.addData("x val: ", x);
                    telemetry.addData("area: ", width * height);
                }
            }   // end for() loop
            telemetry.addData("Buffer time left: ", count);
            telemetry.update();
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            sleep(1);
            count -= 1;
            if (count <= 0) loop = false;
        }



            //telemetry.addData("We going: ", side);
            //telemetry.update();
            //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // TODO: Make sure lines that we need are NOT commented out
                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.85f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    public void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()



}   // end class
