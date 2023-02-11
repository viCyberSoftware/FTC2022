package org.firstinspires.ftc.teamcode.drive.opmode;

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

import org.firstinspires.ftc.teamcode.drive.Robot;
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

        waitForStart();
        runtime.reset();
        robot.init(hardwareMap,gamepad1,gamepad2);
        while (opModeIsActive() && !isStopRequested()) {
            robot.control();
        }
    }
}
