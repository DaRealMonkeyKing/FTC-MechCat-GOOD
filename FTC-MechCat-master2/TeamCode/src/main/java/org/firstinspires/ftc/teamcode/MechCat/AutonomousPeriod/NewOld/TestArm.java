package org.firstinspires.ftc.teamcode.MechCat.AutonomousPeriod.NewOld;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class TestArm extends Autonomous {
    public void runOpMode() {
        // sample mecanum drive for the trajectory
        initialize();
        waitForStart();

        if (isStopRequested()) return;

        if (opModeIsActive()) {
            // run redside roadrunner
            placeBoard();
        }
        // update pose
        PoseStorage.currentPose = drive.getPoseEstimate();

    }   // end runOpMode()
}
