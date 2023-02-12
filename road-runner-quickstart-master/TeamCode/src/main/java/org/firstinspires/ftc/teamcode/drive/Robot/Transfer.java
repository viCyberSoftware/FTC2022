package org.firstinspires.ftc.teamcode.drive.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Transfer {
    public enum State {
        TRANSFER_FRONT,
        TRANSFER_BACK,
        TRANSFER_MOVING_FRONT,
        TRANSFER_MOVING_BACK,
        UNDEFINED
    }

    static final double SERVO_FRONT = 0.05; //de vazut!!!
    static final double SERVO_BACK = 0.05; //devazut!!!

    static final double MOVING_TIME = 2; //in seconds!

    public Servo servoLeft = null;
    public Servo servoRight = null;
    public ElapsedTime timer = null;
    public State state = null;

    public void init(HardwareMap hardwareMap) {
        servoLeft = hardwareMap.get(Servo.class, "transferServoLeft");
        servoRight = hardwareMap.get(Servo.class, "transferServoRight");
        servoRight.setDirection(Servo.Direction.REVERSE);

        timer = new ElapsedTime();
        state = State.UNDEFINED;
        moveFront();
    }

    public void moveFront() {
        if (state != State.TRANSFER_FRONT) {
            servoLeft.setPosition(SERVO_FRONT);
            servoRight.setPosition(SERVO_FRONT);
            state = State.TRANSFER_MOVING_FRONT;
            timer.reset();
        }
    }

    public void moveBack() {
        if (state != State.TRANSFER_BACK) {
            servoLeft.setPosition(SERVO_BACK);
            servoRight.setPosition(SERVO_BACK);
            state = State.TRANSFER_MOVING_BACK;
            timer.reset();
        }
    }

    public void update() {
        if (state == State.TRANSFER_MOVING_FRONT && timer.seconds() >= MOVING_TIME) {
            state = State.TRANSFER_FRONT;
        } else if (state == State.TRANSFER_MOVING_BACK && timer.seconds() >= MOVING_TIME) {
            state = State.TRANSFER_BACK;
        }
    }
}