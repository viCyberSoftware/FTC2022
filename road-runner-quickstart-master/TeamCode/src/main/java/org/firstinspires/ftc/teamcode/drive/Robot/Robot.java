package org.firstinspires.ftc.teamcode.drive.Robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.Robot.Controls;
import org.firstinspires.ftc.teamcode.drive.Robot.Transfer;
import org.firstinspires.ftc.teamcode.drive.Robot.LinearSlide;
import org.firstinspires.ftc.teamcode.drive.Robot.Claw;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Robot {
    /*public static class FSM {
        enum Action {
            IDLE,
            RAISING_TO_LOW_JUNCTION,
            RAISING_TO_MIDDLE_JUNCTION,
            RAISING_TO_HIGH_JUNCTION,
            LOWERING,
        }
        enum Transfer {
            IDLE,
            ROTATING_CLAW,
            TRANSFERRING
        }
        enum Slides {
            IDLE,
            MOVING_UP,
            MOVING_DOWN
        }

        Action action = Action.IDLE;
        Transfer transfer = Transfer.IDLE;
        Slides slides = Slides.IDLE;
    }*/

    public enum Action {
        IDLE,
        RAISING_TO_LOW_JUNCTION,
        RAISING_TO_MIDDLE_JUNCTION,
        RAISING_TO_HIGH_JUNCTION,
        LOWERING
    }

    public Action action = Action.IDLE;

    public ElapsedTime loweringTimeout = null;

    public Transfer transfer = null;
    public Claw claw = null;
    public LinearSlide linearSlide = null;
    public Controls controls = null;
    public SampleMecanumDrive drive = null;
    public Telemetry telemetry = null;

    public void init(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry _telemetry) {
        telemetry = _telemetry;

        claw = new Claw();
        claw.init(hardwareMap);

        transfer = new Transfer();
        transfer.init(hardwareMap);

        linearSlide = new LinearSlide();
        linearSlide.init(hardwareMap);

        controls = new Controls();
        controls.init(gamepad1,gamepad2);

        loweringTimeout = new ElapsedTime();

        initMecanumDrive(hardwareMap);
    }
    public void initPark(HardwareMap hardwareMap){
        claw = new Claw();

        transfer = new Transfer();

        linearSlide = new LinearSlide();
        linearSlide.init(hardwareMap);

        initMecanumDrive(hardwareMap);
    }

    public void raiseToLowJunction() {
        if (claw.openState == Claw.State.CLAW_CLOSED) {
            action = Action.RAISING_TO_LOW_JUNCTION;
            linearSlide.motorsOn();
            linearSlide.moveTo(LinearSlide.State.SLIDER_LOW);
        }
    }

    public void raiseToMiddleJunction(){
        if (claw.openState == Claw.State.CLAW_CLOSED) {
            action = Action.RAISING_TO_MIDDLE_JUNCTION;
            linearSlide.motorsOn();
            linearSlide.moveTo(LinearSlide.State.SLIDER_MIDDLE);
        }
    }

    public void raiseToHighJunction() {
        if (claw.openState == Claw.State.CLAW_CLOSED) {
            action = Action.RAISING_TO_HIGH_JUNCTION;
            ///test
            loweringTimeout.reset();
            linearSlide.motorsOn();
            linearSlide.moveTo(LinearSlide.State.SLIDER_HIGH);
        }
    }

    public void lower() {
        action = Action.LOWERING;
        loweringTimeout.reset();
        linearSlide.motorsOn();
        linearSlide.moveTo(LinearSlide.State.SLIDER_GROUND);
    }

    public void update() {
        claw.update();
        transfer.update();
        linearSlide.update();

        switch (action) {
            case RAISING_TO_LOW_JUNCTION:
            case RAISING_TO_MIDDLE_JUNCTION:
            case RAISING_TO_HIGH_JUNCTION: {
                if (loweringTimeout.seconds() >= 3) {
                    action = Action.IDLE;
                    break;
                }
                claw.rotateDown();
                if (claw.rotationState == Claw.State.CLAW_DOWN
                    && linearSlide.motorsAtTarget()
                    && transfer.state != Transfer.State.TRANSFER_BACK) {
                    transfer.moveBack();
                }
                if (claw.rotationState == Claw.State.CLAW_DOWN
                    && transfer.state == Transfer.State.TRANSFER_BACK
                    && linearSlide.motorsAtTarget()) {
                    action = Action.IDLE;
                }
                break;
            }
            case LOWERING: {
                //timeout
                if (loweringTimeout.seconds() >= 3) {
                    action = Action.IDLE;
                    break;
                }

                if (claw.openState != Claw.State.CLAW_CLOSED) {
                    claw.close();
                } else if (claw.rotationState != Claw.State.CLAW_UP) {
                    claw.rotateUp();
                } else {
                    transfer.moveFront();
                    telemetry.addLine("aici1");
                    if (linearSlide.motorsAtTarget()) {
                        telemetry.addLine("aici2");
                        claw.open();
                        linearSlide.motorsOff();
                        action = Action.IDLE;
                    }
                }
                break;
             }
        }

        //claw.update();
        //transfer.update();
        //linearSlide.update();
    }

    public void control(){
        controls.control(this);
    }

    private void initMecanumDrive(HardwareMap hardwareMap){
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
