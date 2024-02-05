package org.firstinspires.ftc.teamcode.MechCat.ControlledPeriod;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
public class ArmController implements Runnable {
    public int armTarget = 0, slideTarget = 0;
    public int armPosition, slidePosition;
    private int armTargetD = 0, slideTargetD = 0;
    private boolean active = true;
    DcMotorEx LArmMotor, RArmMotor, SlideMotor;
    private static PIDController armController, slideController;

    // PID Values (Fvl = Feed value LONG)
    private static final double armPv = 0.015, armIv = 0, armDv = 0, armFv = 0, armFvl = 0.1,
                                slidePv = 0.001, slideIv = 0, slideDv = 0, slideFv = 0;

    // MAX DISTANCES AND VELOCITIES
    private static final double MAX_SLIDER_DIST = -2200, MAX_ARM_SPEED = 10, MAX_SLIDER_SPEED = 100;

    // Constructor
    public ArmController(DcMotorEx LArm, DcMotorEx RArm, DcMotorEx Slide){
        LArmMotor = LArm;
        RArmMotor = RArm;
        SlideMotor = Slide;

        SlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LArmMotor.setDirection(DcMotorEx.Direction.REVERSE);

        armController = new PIDController(armPv, armIv, armDv);
        slideController = new PIDController(slidePv, slideIv, slideDv);
        armPosition = RArmMotor.getCurrentPosition();
    }

    @Override
    public void run(){
        // running
        while(active){
            armPID();
            slidePID();
        }
    }

    public void PIDactive(boolean bool){
        active = bool;
    }

    public void ArmTo(int Target){
        if (Target < 0){
            armTargetD = 0;
        } else {
            armTargetD = Target;
        }
    }

    public void SlideTo(int Target){
        if (-Target > 0){
            slideTargetD = 0;
        } else if (-Target < MAX_SLIDER_DIST) {
            slideTargetD = (int) MAX_SLIDER_DIST;
        } else{
            slideTargetD = -Target;
        }

    }

    private void armPID(){
        // limit speed
        if (Math.abs(armTargetD - armTarget) <= MAX_ARM_SPEED){
            armTarget = armTargetD;
        } else if (armTargetD > armTarget){
            armTarget += MAX_ARM_SPEED;
        } else if (armTargetD < armTarget){
            armTarget -= MAX_ARM_SPEED;
        }

        // Set Position
        // armController.setPID(armPv, armIv, armDv);
        armPosition = RArmMotor.getCurrentPosition();

        //region PID UPDATING
        double vPID = armController.calculate(armPosition, armTarget);

        //FeedForward Updating
        double feed = (armFvl - armFv) * (slidePosition / MAX_SLIDER_DIST);

        double ticks_per_degree = 1680 / 360f * 2f; // Ticks per rev / 1rev / gear ratio (1:2)

        double vFeed = Math.cos(Math.toRadians((armTarget - 0) / ticks_per_degree)) * feed;

        LArmMotor.setPower(vPID + vFeed);
        RArmMotor.setPower(vPID + vFeed);
        //end code
    }

    private void slidePID(){
        // limit speed
        if (Math.abs(slideTargetD - slideTarget) <= MAX_SLIDER_SPEED){
            slideTarget = slideTargetD;
        } else if (slideTargetD > slideTarget){
            slideTarget += MAX_SLIDER_SPEED;
        } else if (slideTargetD < slideTarget){
            slideTarget -= MAX_SLIDER_SPEED;
        }

        // Set Position
        // slideController.setPID(slidePv, slideIv, slideDv);
        slidePosition = SlideMotor.getCurrentPosition();

        //region PID UPDATING
        double vPID = slideController.calculate(slidePosition, slideTarget);

        double ticks_per_degree = 3.84444444444444444444444444444;

        double vFeed = Math.cos(Math.toRadians((slideTarget - 0) / ticks_per_degree)) * slideFv;

        SlideMotor.setPower(vPID + vFeed);
        //end code
    }
}
