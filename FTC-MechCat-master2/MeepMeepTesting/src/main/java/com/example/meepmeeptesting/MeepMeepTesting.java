package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        Pose2d startPos =new Pose2d(12, -60, Math.toRadians(90));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(54, 50, Math.toRadians(312), Math.toRadians(312), 13)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPos)
                                .lineToLinearHeading(new Pose2d(34, -32, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(48, -41, Math.toRadians(180)))
                                .strafeLeft(19)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    // red side back starting position
    /*
    drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5, -60, Math.toRadians(90)))
                                .forward(24)
                                .back(24)
                                .strafeLeft(23)
                                .forward(30)
                                .splineToLinearHeading(new Pose2d(35, -5, Math.toRadians(180)), Math.toRadians(0))
                                .strafeTo(new Vector2d(48, -36))
                                .forward(8)
                                .splineToLinearHeading(new Pose2d(58, -9, Math.toRadians(180)), Math.toRadians(0))
                                .build()
                );
     // blue side back starting position
     drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35.5, 60, Math.toRadians(270)))
                                .forward(24)
                                .back(24)
                                .strafeRight(23)
                                .forward(30)
                                .splineToLinearHeading(new Pose2d(35, 5, Math.toRadians(180)), Math.toRadians(0))
                                .strafeTo(new Vector2d(48, 36))
                                .forward(8)
                                .splineToLinearHeading(new Pose2d(58, 9, Math.toRadians(180)), Math.toRadians(0))
                                .build()
                );

        // blue side right starting position
        drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 60, Math.toRadians(270)))
                                .forward(24)
                                .back(24)
                                .strafeLeft(12)
                                .splineToLinearHeading(new Pose2d(38, 36, Math.toRadians(180)), Math.toRadians(90))
                                .back(10)
                                .forward(8)
                                .splineToLinearHeading(new Pose2d(58, 60, Math.toRadians(180)), Math.toRadians(0))
                                .build()

            drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
                                .forward(24)
                                .lineToLinearHeading(new Pose2d(-36, -54, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-34, -42, Math.toRadians(60)))
                                .lineToLinearHeading(new Pose2d(-36, -54, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-38, -42, Math.toRadians(120)))
                                .lineToLinearHeading(new Pose2d(-36, -54, Math.toRadians(90)))
                                .strafeLeft(10)
                                .splineToConstantHeading(new Vector2d(-58, -10), Math.toRadians(90))
                                .strafeRight(69)
                                .splineTo(new Vector2d(38, -35), Math.toRadians(0))
                                .back(10)
                                .forward(8)
                                .splineToConstantHeading(new Vector2d(58, -60), Math.toRadians(0))
                                .build()

     */
}