package org.firstinspires.ftc.teamcode.MechCat.ControlledPeriod.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ClawTuner")
@Config
public class ClawTuning extends OpMode {
    public static double claw1pos = 0.6;
    public static double claw2pos = 0;
    public static double clawServopos = 1;
    Servo claw1, claw2, clawServo;

    @Override
    public void init() {
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    @Override
    public void loop() {
        claw1.setPosition(claw1pos);
        claw2.setPosition(claw2pos);
        clawServo.setPosition(clawServopos);
        telemetry.addData("claw1 pos:", claw1.getPosition());
        telemetry.addData("claw2 pos:", claw2.getPosition());
        telemetry.update();
    }
}