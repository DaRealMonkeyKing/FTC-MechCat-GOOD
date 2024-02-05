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

import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

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

//@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutonomousAudience")
public class AutonomousAudience extends Autonomous {

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
        resetRuntime();
        if (isStopRequested()) return;

        if (opModeIsActive()) {
            // run redside roadrunner
            getSide();
            telemetry.addData("Side", side);
            RRRoadRunner(drive, side);
        }
    }

    public void RRRoadRunner(SampleMecanumDrive drive, String whatSidussy){
        boolean lBoard = false, mBoard = false, rBoard = false;
        // ROADRUNNER RED RIGHT-SIDE
        // increasing heading goes counterclockwise
        double addAngle = Math.toRadians(180);
        double invert = -1;
        double addY = 0;

        // Run the correct side blah blah BORING
        if (whatSidussy.charAt(0) == 'R'){  // RED SIDE
            invert = -1;
            addAngle = Math.toRadians(0);
            addY = 0;
            Pose2d startPos = new Pose2d(-36, 60 * invert, Math.toRadians(90) + addAngle);
            drive.setPoseEstimate(startPos);
            if (whatSidussy.charAt(1) == 'R')
                runBoardSide(drive, invert, addY, startPos);
            else if (whatSidussy.charAt(1) == 'L')
                runAudienceSide(drive, invert, addY, startPos);
            else
                runMid(drive, invert, addY, startPos);
        } else if (whatSidussy.charAt(0) == 'B'){                            // BLUE SIDE
            invert = 1;
            addAngle = Math.toRadians(180);
            addY = 70;
            Pose2d startPos = new Pose2d(-36, 60 * invert, Math.toRadians(90) + addAngle);
            drive.setPoseEstimate(startPos);
            if (whatSidussy.charAt(1) == 'L')
                runBoardSide(drive, invert, addY, startPos);
            else if (whatSidussy.charAt(1) == 'R')
                runAudienceSide(drive, invert, addY, startPos);
            else
                runMid(drive, invert, addY, startPos);
        }
        // update pose
        PoseStorage.currentPose = drive.getPoseEstimate();
    }

// BOARD SIDE IS THE SPIKE CLOSEST TO THE  BOARD
// Audience side is the spike closest to the audience
// Red and blue are flipped btw so blue's right side would be red's left side.
// TODO: check / remake values for the trajectories as you see fit
    private void runBoardSide(SampleMecanumDrive drive, double invert, double addY, Pose2d startpos){
        Trajectory goToPixel = drive.trajectoryBuilder(startpos)
                .splineToSplineHeading(new Pose2d(-31, 35 * invert, Math.toRadians(0)), Math.toRadians(340))
                .build();

        Trajectory goToWhitePixel = drive.trajectoryBuilder((goToPixel.end()))
                .lineToLinearHeading(new Pose2d(-55, 35 * invert, Math.toRadians(180)))
                .build();

        Trajectory goBackBackBack = drive.trajectoryBuilder(goToWhitePixel.end())
                .lineToLinearHeading(new Pose2d(-55, 34 * invert, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(40, -15), Math.toRadians(350))
                .build();

        Trajectory goToBack = drive.trajectoryBuilder(goBackBackBack.end())
                .lineToLinearHeading(new Pose2d(50, -42 + addY, Math.toRadians(180)))
                .build();

        Trajectory goPark = drive.trajectoryBuilder(goToBack.end())
                .forward(8)
                .splineToConstantHeading(new Vector2d(54, 11 * invert), Math.toRadians(0))
                .build();

        drive.followTrajectory(goToPixel);
        placePixelLine();
        drive.followTrajectory(goToWhitePixel);
        drive.followTrajectory(goBackBackBack);
        drive.followTrajectory(goToBack);
        placeBoard();

        // cycle(drive, invert, goToBack);

        drive.followTrajectory(goPark);
        drive.update();
    }
    // TODO: check / remake values for the trajectories as you see fit
    private void runMid(SampleMecanumDrive drive, double invert, double addY, Pose2d startpos){
        Trajectory goToPixel = drive.trajectoryBuilder(startpos)
                .forward(27)
                .build();

        Trajectory goToWhitePixel = drive.trajectoryBuilder((goToPixel.end()))
                .lineToLinearHeading(new Pose2d(-55, 35 * invert, Math.toRadians(180)))
                .build();

        Trajectory goBackBackBack = drive.trajectoryBuilder(goToWhitePixel.end())
                .lineToLinearHeading(new Pose2d(-55, 34 * invert, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(40, invert * 15), Math.toRadians(350))
                .build();

        Trajectory goToBack = drive.trajectoryBuilder(goBackBackBack.end())
                .lineToLinearHeading(new Pose2d(48.5, -36 + addY, Math.toRadians(180)))
                .build();

        Trajectory goPark = drive.trajectoryBuilder(goToBack.end())
                .forward(8)
                .splineToConstantHeading(new Vector2d(54, invert * 11), Math.toRadians(0))
                .build();

        clawServo.setPosition(ClawServoGround);
        drive.followTrajectory(goToPixel);
        placePixelLine();
        drive.followTrajectory(goToWhitePixel);
        drive.followTrajectory(goBackBackBack);
        drive.followTrajectory(goToBack);
        placeBoard();

        // cycle(drive, invert, goToBack);

        drive.followTrajectory(goPark);
        drive.update();
    }
    // TODO: check / remake values for the trajectories as you see fit
    private void runAudienceSide(SampleMecanumDrive drive, double invert, double addY, Pose2d startpos){
        Trajectory goToPixel = drive.trajectoryBuilder(startpos)
                .splineToSplineHeading(new Pose2d(-31, 35 * invert, Math.toRadians(180)), Math.toRadians(160))
                .build();

        Trajectory goToWhitePixel = drive.trajectoryBuilder((goToPixel.end()))
                .lineToLinearHeading(new Pose2d(-55, 35 * invert, Math.toRadians(180)))
                .build();

        Trajectory goBackBackBack = drive.trajectoryBuilder(goToWhitePixel.end())
                .lineToLinearHeading(new Pose2d(-55, 34 * invert, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(40, -15), Math.toRadians(350))
                .build();

        Trajectory goToBack = drive.trajectoryBuilder(goBackBackBack.end())
                .lineToLinearHeading(new Pose2d(50, -31.5 + addY, Math.toRadians(180)))
                .build();

        Trajectory goPark = drive.trajectoryBuilder(goToBack.end())
                .forward(8)
                .splineToConstantHeading(new Vector2d(54, 11 * invert), Math.toRadians(0))
                .build();

        drive.followTrajectory(goToPixel);
        placePixelLine();
        drive.followTrajectory(goToWhitePixel);
        drive.followTrajectory(goBackBackBack);
        drive.followTrajectory(goToBack);
        placeBoard();

        // cycle(drive, invert, goToBack);

        drive.followTrajectory(goPark);
        drive.update();
    }

    public void cycle(SampleMecanumDrive drive, double invert, Trajectory startpos) {
        // move to center of backdrop
        Trajectory goToBack = drive.trajectoryBuilder(startpos.end())
                .lineTo(new Vector2d(48, 33 * invert))
                .build();

        double time = getRuntime();
        boolean loop = true;
        // callable function for one cycle of grabbing pixel and dropping
            while (loop) {
                if (!opModeIsActive() || time > 20)
                    break;
                Score(drive, invert, goToBack);
                time = getRuntime();
            }

         // while less than 20000 ms, go for another cycle
    }

    public void Score(SampleMecanumDrive drive, double invert, Trajectory startpos) {
        // go to pixel, grab, go back and score
        int localSpdLMT = 40;

        // TODO: Traj
        Trajectory leaveBoard = drive.trajectoryBuilder(startpos.end())
                .lineToLinearHeading(new Pose2d(30, 10 * invert, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(localSpdLMT, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory goToPixel = drive.trajectoryBuilder(leaveBoard.end())
                .lineToLinearHeading(new Pose2d(-55.5, 10 * invert, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory posFirstPixel = drive.trajectoryBuilder(goToPixel.end())
                .lineToLinearHeading(new Pose2d(-55.8, 13.65 * invert, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(localSpdLMT, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory backup = drive.trajectoryBuilder(posFirstPixel.end())
                .back(10)
                .build();

        TrajectorySequence turn = drive.trajectorySequenceBuilder(posFirstPixel.end())
                .turn(Math.toRadians(45))
                .build();

        TrajectorySequence turnBack = drive.trajectorySequenceBuilder(posFirstPixel.end())
                .turn(Math.toRadians(-45))
                .build();

        Trajectory backToPixel = drive.trajectoryBuilder(posFirstPixel.end())
                .lineToLinearHeading(new Pose2d(-47, 10 * invert, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory goToBoard = drive.trajectoryBuilder(backToPixel.end())
                .lineToLinearHeading(new Pose2d(30, 10 * invert, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory goBoard = drive.trajectoryBuilder(goToBoard.end())
                .lineToLinearHeading(new Pose2d(48, 33 * invert, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(localSpdLMT, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        drive.followTrajectory(leaveBoard);

        drive.followTrajectory(goToPixel);
        sayAAAAH();

        drive.followTrajectory(posFirstPixel);
        Clamp();

        drive.followTrajectorySequence(turn);
        drive.followTrajectorySequence(turnBack);

        //drive.followTrajectory(backup);
        drive.followTrajectory(backToPixel);

        drive.followTrajectory(goToBoard);
        drive.followTrajectory(goBoard);

        clawServo.setPosition(ClawServoBoard);
        placeBoard();
        
    }

    public void sayAAAAH(){
        claw1.setPosition(Claw1OpenPos);
        claw2.setPosition(Claw2OpenPos);
        sleep(150);
    }

    public void Clamp(){
        clawServo.setPosition(ClawServoGround);
        sleep(1000);
        claw1.setPosition(Claw1ClosePos);
        claw2.setPosition(Claw2ClosePos);
        sleep(100);
    }

}   // end class
