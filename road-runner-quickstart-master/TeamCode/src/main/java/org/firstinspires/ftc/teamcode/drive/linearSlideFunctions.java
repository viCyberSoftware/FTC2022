package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


public class linearSlideFunctions {

    public static DcMotorEx motorSlideLeft = null;
    public static DcMotorEx motorSlideRight = null;
    public static Servo servo = null;
    public static Servo servo1 = null;
    public static Servo servoGheara = null;
    public static int initialPosition = 0;

    public static void init(HardwareMap hardwareMap) {

        servoGheara = hardwareMap.get(Servo.class, "servoGheara");
        servoGheara.setPosition(0.75);
        servo = hardwareMap.get(Servo.class, "servo0");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servoGheara = hardwareMap.get(Servo.class, "servoGheara");
        motorSlideLeft = hardwareMap.get(DcMotorEx.class, "motorSlideLeft");
        motorSlideRight = hardwareMap.get(DcMotorEx.class, "motorSlideRight");

        motorSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        initialPosition = motorSlideLeft.getCurrentPosition();
        motorSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlideRight.setPower(0.5);
        motorSlideLeft.setPower(0.5);
        motorSlideLeft.setTargetPosition(motorSlideLeft.getCurrentPosition());
        motorSlideRight.setTargetPosition(motorSlideRight.getCurrentPosition());
        servo.setPosition(0.5);
        servo1.setPosition(0.5);
        servoGheara.setPosition(0.75);
    }

    public static void move(Gamepad gamepad1) {

        double pos0 = 0.75, pos1 = 0.5;
        boolean up_pressed = false;
        boolean down_pressed = false;
        boolean a_pressed = false;
        boolean b_pressed = false;

        if (gamepad1.dpad_up) {
            if (!up_pressed) {
                pos0 += 0.03;
                //   pos1 -= 0.03;
            }
           // up_pressed = true;
            telemetry.addData("pos0", pos0);
            servoGheara.setPosition(pos0);
            //    telemetry.addData("servoPos", servo.getPosition());
            telemetry.update();
            //   servo1.setPosition(pos1);
            //linearSlideFunctions.moveUp();
        }

        if (gamepad1.dpad_down) {
            if (!down_pressed) {
                pos0 -= 0.03;
                //  pos1 += 0.03;
                servoGheara.setPosition(pos0);

                //   servo.setPosition(pos0);
                // servo1.setPosition(pos1);
            }
            //down_pressed = true;
            // linearSlideFunctions.moveDown();
        }

        if (gamepad1.a) {
            if (!a_pressed && motorSlideLeft.getCurrentPosition() - initialPosition <= 2200) {
                motorSlideLeft.setPower(0.5);
                motorSlideRight.setPower(0.5);
                motorSlideLeft.setTargetPosition(motorSlideLeft.getCurrentPosition() + 10);
                motorSlideRight.setTargetPosition(motorSlideRight.getCurrentPosition() + 10);
            }
        }

        if (gamepad1.b) {
            if (motorSlideLeft.getCurrentPosition() - initialPosition >= 10) {
                motorSlideLeft.setPower(0.5);
                motorSlideRight.setPower(0.5);
                motorSlideLeft.setTargetPosition(motorSlideLeft.getCurrentPosition() - 10);
                motorSlideRight.setTargetPosition(motorSlideRight.getCurrentPosition() - 10);
            } else if (!b_pressed) {
                motorSlideLeft.setPower(0);
                motorSlideRight.setPower(0);
            }


        }
    }
}

