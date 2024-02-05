package org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.List;

public class REDR extends NewAutonomous{
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
            RunRoadRunner(drive, side);
        }
        arm.PIDactive(false);
        PID.stop();
    }

    public void RunRoadRunner(SampleMecanumDrive drive, String side) {
        // START POS
        Pose2d startPos = new Pose2d(12, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPos);
        // GO TO PROP
        // goToProp();
        goToProp(drive, side.charAt(0), startPos);
        // GO TO BACK
        // goToBack();
        goToBack(drive, side.charAt(0), trajs.get(trajs.size() - 1));
        // PARK??
        // park();
        park(drive, side.charAt(0), trajs.get(trajs.size() - 1));
        /*
        * setArm(int TARGET) NO MAX
        * setSlider(int TARGET) MAXES OUT AT 2200
        * Both min = 0
        */
    }

    public void goToProp(SampleMecanumDrive drive, char side, Pose2d startpos){
//      trajs.get(trajs.size() - 1);
//      trajs.add(OURTRAJ);

        Trajectory left = drive.trajectoryBuilder(startpos)
                .lineToLinearHeading(new Pose2d(12, -35, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(spdLMT, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory mid = drive.trajectoryBuilder(startpos)
                .lineToLinearHeading(new Pose2d(12, -35, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(spdLMT, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory right = drive.trajectoryBuilder(startpos)
                .lineToLinearHeading(new Pose2d(12, -35, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(spdLMT, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory right2 = drive.trajectoryBuilder(startpos)
                .lineToLinearHeading(new Pose2d(34, -32, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(spdLMT, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();


        clawServoDown(true);

        switch(side) {
            case 'R':
                drive.followTrajectory(right);
                sleep(500);
                drive.followTrajectory(right2);
                trajs.add(right);
                break;
            case 'M':
                drive.followTrajectory(mid);
                trajs.add(mid);
                break;
            case 'L':
                drive.followTrajectory(left);
                trajs.add(left);
                break;

        }

        placePixelLine();
    }

    public void goToBack(SampleMecanumDrive drive, char side, Trajectory startpos) {
        Trajectory toRightBack = drive.trajectoryBuilder(startpos.end())
                .lineToLinearHeading(new Pose2d(48, -41, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(spdLMT, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory toLeftBack = drive.trajectoryBuilder(startpos.end())
                .lineToLinearHeading(new Pose2d(49, -32, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(spdLMT, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory toMidBack = drive.trajectoryBuilder(startpos.end())
                .lineToLinearHeading(new Pose2d(49, -36, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(spdLMT, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        switch(side) {
            case 'R':
                drive.followTrajectory(toRightBack);
                trajs.add(toRightBack);
                break;
            case 'M':
                drive.followTrajectory(toMidBack);
                trajs.add(toMidBack);
                break;
            case 'L':
                drive.followTrajectory(toLeftBack);
                trajs.add(toLeftBack);
                break;

        }

        // score
    }

    public void park(SampleMecanumDrive drive, char side, Trajectory startpos) {
        double length = 22;
        switch(side) {
            case 'R':
                length = 19;
                break;
            case 'M':
                length = 22;
                break;
            case 'L':
                length = 25;
                break;

        }

        Trajectory goPark = drive.trajectoryBuilder(startpos.end())
                .strafeLeft(length,
                        SampleMecanumDrive.getVelocityConstraint(spdLMT, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        drive.followTrajectory(goPark);
    }
}
