package org.firstinspires.ftc.teamcode.drive.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Transfer {
    public enum State {
        TRANSFER_FRONT,
        TRANSFER_BACK,
        TRANSFER_PARK,
        TRANSFER_MOVING_FRONT,
        TRANSFER_MOVING_BACK,
        UNDEFINED
    }

    static final double SERVO_FRONT = 0.028; //de vazut!!!
    static final double SERVO_BACK = 0.135; //devazut!!! 0.15 initial
    static final double SERVO_PARK = 0.035;

    static final double MOVING_TIME = 0.2; //in seconds!

    public Servo servoLeft = null;
    public Servo servoRight = null;
    public ElapsedTime timer = null;
    public State state;

    public void init(HardwareMap hardwareMap) {
        servoLeft = hardwareMap.get(Servo.class, "transferServoLeft");
        servoRight = hardwareMap.get(Servo.class, "transferServoRight");
        servoRight.setDirection(Servo.Direction.REVERSE);

        timer = new ElapsedTime();
        state = State.UNDEFINED;
        moveFront();
    }

    public void moveFront() {
        if (state != State.TRANSFER_FRONT && state != State.TRANSFER_MOVING_FRONT) {
            servoLeft.setPosition(SERVO_FRONT);
            servoRight.setPosition(SERVO_FRONT);
            state = State.TRANSFER_MOVING_FRONT;
            timer.reset();
        }
    }


    public void moveBack() {
        if (state != State.TRANSFER_BACK && state != State.TRANSFER_MOVING_BACK) {
            servoLeft.setPosition(SERVO_BACK);
            servoRight.setPosition(SERVO_BACK);
            state = State.TRANSFER_MOVING_BACK;
            timer.reset();
        }
    }

    public void movePark(){
        if(state != State.TRANSFER_PARK) {
            servoLeft.setPosition(SERVO_PARK);
            servoRight.setPosition(SERVO_PARK);
            state = State.TRANSFER_PARK;
            timer.reset();
        }
    }
    ////de test
    public void moveBackTesting(){
        servoLeft.setPosition(servoLeft.getPosition()-0.005);
        servoRight.setPosition((servoRight.getPosition()-0.005));
    }
    public void moveFrontTesting(){
        servoLeft.setPosition(servoLeft.getPosition()+0.005);
        servoRight.setPosition((servoRight.getPosition()+0.005));
    }

    public void update() {
        if (state == State.TRANSFER_MOVING_FRONT && timer.seconds() >= MOVING_TIME) {
            state = State.TRANSFER_FRONT;
        }
        if (state == State.TRANSFER_MOVING_BACK && timer.seconds() >= MOVING_TIME) {
            state = State.TRANSFER_BACK;
        }
    }
}