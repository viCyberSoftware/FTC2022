package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.Robot.Claw;
import org.firstinspires.ftc.teamcode.drive.Robot.Robot;
import org.firstinspires.ftc.teamcode.drive.Robot.Transfer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.linearSlideFunctions;

import org.firstinspires.ftc.teamcode.util.Encoder;


@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class BasicOpMode_Linear extends LinearOpMode {


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot = new Robot();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

//        linearSlideFunctions.init(hardwareMap);
        robot.init(hardwareMap,gamepad1,gamepad2, telemetry);
        waitForStart();
        runtime.reset();
        while (opModeIsActive() && !isStopRequested()) {
            robot.control();
            if (robot.claw.openState == Claw.State.CLAW_OPEN) {
                telemetry.addLine("claw open");
            }
            if (robot.claw.openState == Claw.State.CLAW_OPENING) {
                telemetry.addLine("claw opening");
            }
            if (robot.claw.openState == Claw.State.CLAW_CLOSING) {
                telemetry.addLine("claw closing");
            }
            if (robot.claw.openState == Claw.State.CLAW_CLOSED) {
                telemetry.addLine("claw closed");
            }
            if (robot.action == Robot.Action.IDLE) {
                telemetry.addLine("idle");
            }
            if (robot.action == Robot.Action.LOWERING) {
                telemetry.addLine("lowering");
            }
            if (robot.action == Robot.Action.RAISING_TO_HIGH_JUNCTION) {
                telemetry.addLine("raising to high");
            }
            if (robot.action == Robot.Action.RAISING_TO_LOW_JUNCTION) {
                telemetry.addLine("raising to low");
            }
            if (robot.action == Robot.Action.RAISING_TO_MIDDLE_JUNCTION) {
                telemetry.addLine("raising to middle");
            }
            if (robot.linearSlide.motorsAtTarget()) {
                telemetry.addLine("motors at target");
            }
            if (robot.claw.rotationState == Claw.State.CLAW_ROTATING_UP) {
                telemetry.addLine("claw rotating up");
            }
            if (robot.claw.rotationState == Claw.State.CLAW_ROTATING_DOWN) {
                telemetry.addLine("claw rotating down");
            }
            if (robot.claw.rotationState == Claw.State.CLAW_UP) {
                telemetry.addLine("claw up");
            }
            if (robot.claw.rotationState == Claw.State.CLAW_DOWN) {
                telemetry.addLine("claw down");
            }
            if (robot.transfer.state == Transfer.State.TRANSFER_MOVING_BACK) {
                telemetry.addLine("transfer moving back");
            }
            if (robot.transfer.state == Transfer.State.TRANSFER_MOVING_FRONT) {
                telemetry.addLine("transfer moving front");
            }
            if (robot.transfer.state == Transfer.State.TRANSFER_BACK) {
                telemetry.addLine("transfer back");
            }
            if (robot.transfer.state == Transfer.State.TRANSFER_FRONT) {
                telemetry.addLine("transfer front");
            }
            telemetry.addData("target ticks: ", robot.linearSlide.motorSlideRight.getTargetPosition());
            telemetry.addData("current tick: ", robot.linearSlide.motorSlideLeft.getCurrentPosition());
            telemetry.addData("position of transfer",robot.transfer.servoLeft.getPosition());
            telemetry.addData("timer closing: ", robot.claw.timerClosing.seconds());
            telemetry.addData("position of fastenClaw",robot.claw.servoFastenClaw.getPosition());
            telemetry.update();

        }
    }
}
