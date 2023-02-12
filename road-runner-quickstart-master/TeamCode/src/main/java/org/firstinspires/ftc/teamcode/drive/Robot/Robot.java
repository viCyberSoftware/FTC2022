package org.firstinspires.ftc.teamcode.drive.Robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.Robot.Controls;
import org.firstinspires.ftc.teamcode.drive.Robot.Transfer;
import org.firstinspires.ftc.teamcode.drive.Robot.LinearSlide;
import org.firstinspires.ftc.teamcode.drive.Robot.Claw;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Robot {
    public Transfer transfer = null;
    public Claw claw = null;
    public LinearSlide linearSlide = null;
    public Controls controls = null;
    public SampleMecanumDrive drive = null;

    public void init(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        claw = new Claw();
        claw.init(hardwareMap);

        transfer = new Transfer();
        transfer.init(hardwareMap);

        linearSlide = new LinearSlide();
        linearSlide.init(hardwareMap);

        controls = new Controls();
        controls.init(gamepad1,gamepad2);

        initMecanumDrive(hardwareMap,gamepad1,gamepad2);
    }

    public void update() {
        claw.update();
        transfer.update();
        linearSlide.update();
    }

    public void control(){
        controls.control(this);
    }

    private void initMecanumDrive(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2){
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
