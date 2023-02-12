package org.firstinspires.ftc.teamcode.drive.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {
    public enum State {
        CLAW_OPEN,
        CLAW_CLOSED,
        CLAW_OPENING,
        CLAW_CLOSING,

        CLAW_UP,
        CLAW_DOWN,
        CLAW_ROTATING_UP,
        CLAW_ROTATING_DOWN,

        UNDEFINED
    }

    static final double SERVO_OPEN = 0.8; //de updatat!!!
    static final double SERVO_CLOSED = 1; //de updatat!!!
    static final double SERVO_UP = 0; //de updatat!!!
    static final double SERVO_DOWN = 1.0; //de updatat!!!

    static final double CLOSING_TIME = 2; //in seconds!
    static final double ROTATING_TIME = 2; //in seconds!

    public Servo servoFastenClaw = null;
    public Servo servoRotateClaw = null;
    public State openState = null;
    public State rotationState = null;
    public ElapsedTime timerClosing = null;
    public ElapsedTime timerRotating = null;

    public void init(HardwareMap hardwareMap) {
        servoFastenClaw = hardwareMap.get(Servo.class, "servoFastenClaw");
        servoRotateClaw = hardwareMap.get(Servo.class,"servoRotateClaw");

        timerClosing = new ElapsedTime();
        timerRotating = new ElapsedTime();

        openState = Claw.State.UNDEFINED;
        rotationState = Claw.State.UNDEFINED;

        close();
        rotateUp();
    }

    public void update() {

        if (openState == State.CLAW_OPENING) {
            if (timerClosing.seconds() >= CLOSING_TIME) {
                openState = State.CLAW_OPEN;
            }
        } else if (openState == State.CLAW_CLOSING) {
            if (timerClosing.seconds() >= CLOSING_TIME) {
                openState = State.CLAW_CLOSED;
            }
        }

        if (rotationState == State.CLAW_ROTATING_UP) {
            if (timerRotating.seconds() >= ROTATING_TIME) {
                rotationState = State.CLAW_UP;
            }
        } else if (rotationState == State.CLAW_ROTATING_DOWN) {
            if (timerRotating.seconds() >= ROTATING_TIME) {
                rotationState = State.CLAW_DOWN;
            }
        }
    }

    public void rotateUp() {
        if (rotationState != State.CLAW_ROTATING_UP && rotationState != State.CLAW_UP) {
            servoRotateClaw.setPosition(SERVO_UP);
            rotationState = State.CLAW_ROTATING_UP;
            timerRotating.reset();
        }
    }

    public void rotateDown() {
        if (rotationState != State.CLAW_ROTATING_DOWN && rotationState != State.CLAW_DOWN) {
            servoRotateClaw.setPosition(SERVO_DOWN);
            rotationState = State.CLAW_ROTATING_DOWN;
            timerRotating.reset();
        }
    }

    public void open() {
        if (openState != State.CLAW_OPEN && openState != State.CLAW_OPENING) {
            servoFastenClaw.setPosition(SERVO_OPEN);
            openState = State.CLAW_OPENING;
            timerClosing.reset();
        }
    }

    public void close() {
        if (openState != State.CLAW_CLOSED && openState != State.CLAW_CLOSING) {
            servoFastenClaw.setPosition(SERVO_CLOSED);
            openState = State.CLAW_CLOSING;
            timerClosing.reset();
        }
    }

    public boolean canTransfer() {
        return openState == State.CLAW_CLOSED &&
               (rotationState == State.CLAW_UP || rotationState == State.CLAW_DOWN);
    }
}