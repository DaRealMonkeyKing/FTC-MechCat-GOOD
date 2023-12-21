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

package org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod;

import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.Claw2ClosePos;
import static org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.AutoConstants.ClawServoBoard;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutonomousBoard")
public class AutonomousBoard extends Autonomous {

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();
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
            Pose2d startPos = new Pose2d(12, 60 * invert, Math.toRadians(90) + addAngle);
            drive.setPoseEstimate(startPos);
            if (whatSidussy.charAt(1) == 'R')
                runBoardSide(drive, invert, addY, startPos);
            else if (whatSidussy.charAt(1) == 'L')
                runAudienceSide(drive, invert, addY, startPos);
            else
                runMid(drive, invert, addY, startPos);
        } else {                            // BLUE SIDE
            invert = 1;
            addAngle = Math.toRadians(180);
            addY = 70;
            Pose2d startPos = new Pose2d(12, 60 * invert, Math.toRadians(90) + addAngle);
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
                .splineToSplineHeading(new Pose2d(33, 33 * invert, Math.toRadians(180)), Math.toRadians(330))
                .build();

        Trajectory goToBack = drive.trajectoryBuilder(goToPixel.end())
                .lineToLinearHeading(new Pose2d(50, -42 + addY, Math.toRadians(180)))
                .build();

        Trajectory goPark = drive.trajectoryBuilder(goToBack.end())
                .forward(8)
                .splineToConstantHeading(new Vector2d(54, 61.5 * invert), Math.toRadians(0))
                .build();

        drive.followTrajectory(goToPixel);
        placePixelLine();
        drive.followTrajectory(goToBack);
        placeBoard();
        drive.followTrajectory(goPark);
        drive.update();
    }
    // TODO: check / remake values for the trajectories as you see fit
    private void runMid(SampleMecanumDrive drive, double invert, double addY, Pose2d startpos){
        Trajectory goToPixel = drive.trajectoryBuilder(startpos)
                .forward(24)
                .build();

        Trajectory goToBack = drive.trajectoryBuilder(goToPixel.end())
                .lineToLinearHeading(new Pose2d(50, -36 + addY, Math.toRadians(180)))
                .build();

        Trajectory goPark = drive.trajectoryBuilder(goToBack.end())
                .forward(8)
                .splineToConstantHeading(new Vector2d(54, 61.5 * invert), Math.toRadians(0))
                .build();

        drive.followTrajectory(goToPixel);
        placePixelLine();
        drive.followTrajectory(goToBack);
        placeBoard();
        drive.followTrajectory(goPark);
        drive.update();
    }
    // TODO: check / remake values for the trajectories as you see fit
    private void runAudienceSide(SampleMecanumDrive drive, double invert, double addY, Pose2d startpos){
        Trajectory goToPixel = drive.trajectoryBuilder(startpos)
                .splineToSplineHeading(new Pose2d(7, (35 * invert), Math.toRadians(180)), Math.toRadians(160))
                .build();

        Trajectory goToBack = drive.trajectoryBuilder(goToPixel.end())
                .lineToLinearHeading(new Pose2d(50, -31.5 + addY, Math.toRadians(180)))
                .build();

        Trajectory goPark = drive.trajectoryBuilder(goToBack.end())
                .forward(8)
                .splineToConstantHeading(new Vector2d(54, 61.5 * invert), Math.toRadians(0))
                .build();

        drive.followTrajectory(goToPixel);
        placePixelLine();
        drive.followTrajectory(goToBack);
        placeBoard();
        drive.followTrajectory(goPark);
        drive.update();
    }

}   // end class
