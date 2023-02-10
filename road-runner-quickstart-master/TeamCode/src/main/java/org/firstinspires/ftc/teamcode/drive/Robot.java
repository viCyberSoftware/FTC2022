package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {
    public Transfer transfer = null;
    public Claw claw = null;

    public static class Transfer {
        public enum State {
            TRANSFER_FRONT,
            TRANSFER_BACK,
            TRANSFER_MOVING_FRONT,
            TRANSFER_MOVING_BACK
        }

        static final double SERVO_FRONT = 0; //de vazut!!!
        static final double SERVO_BACK = 0; //devazut!!!

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

            moveFront();
        }

        public void moveFront() {
            servoLeft.setPosition(SERVO_FRONT);
            servoRight.setPosition(SERVO_FRONT);
            state = State.TRANSFER_MOVING_FRONT;
            timer.reset();
        }

        public void moveBack() {
            servoLeft.setPosition(SERVO_BACK);
            servoRight.setPosition(SERVO_BACK);
            state = State.TRANSFER_MOVING_BACK;
            timer.reset();
        }

        public void update() {
            if (state == State.TRANSFER_MOVING_FRONT && timer.seconds() >= MOVING_TIME) {
                state = State.TRANSFER_FRONT;
            } else if (state == State.TRANSFER_MOVING_BACK && timer.seconds() >= MOVING_TIME) {
                state = State.TRANSFER_BACK;
            }
        }
    }
    public static class Claw {
        public enum State {
            CLAW_OPEN,
            CLAW_CLOSED,
            CLAW_OPENING,
            CLAW_CLOSING,

            CLAW_UP,
            CLAW_DOWN,
            CLAW_ROTATING_UP,
            CLAW_ROTATING_DOWN,
        };

        static final double SERVO_OPEN = 0; //de updatat!!!
        static final double SERVO_CLOSED = 0.8; //de updatat!!!
        static final double SERVO_UP = 0; //de updatat!!!
        static final double SERVO_DOWN = 0.8; //de updatat!!!

        static final double CLOSING_TIME = 2; //in seconds!
        static final double ROTATING_TIME = 2; //in seconds!

        public Servo servo = null;
        public State openState = null;
        public State rotationState = null;
        public ElapsedTime timerClosing = null;
        public ElapsedTime timerRotating = null;

        public void init(HardwareMap hardwareMap) {
            servo = hardwareMap.get(Servo.class, "clawServo");

            timerClosing = new ElapsedTime();
            timerRotating = new ElapsedTime();

            openState = Claw.State.CLAW_CLOSED;
            rotationState = Claw.State.CLAW_UP;

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
            servo.setPosition(SERVO_UP);
            rotationState = State.CLAW_ROTATING_UP;
            timerRotating.reset();
        }

        public void rotateDown() {
            servo.setPosition(SERVO_DOWN);
            rotationState = State.CLAW_ROTATING_DOWN;
            timerRotating.reset();
        }

        public void open() {
            servo.setPosition(SERVO_OPEN);
            openState = State.CLAW_OPENING;
            timerClosing.reset();
        }

        public void close() {
            servo.setPosition(SERVO_CLOSED);
            openState = State.CLAW_CLOSING;
            timerClosing.reset();
        }
    }

    public void init(HardwareMap hardwareMap) {
        claw = new Claw();
        transfer = new Transfer();

        claw.init(hardwareMap);
        transfer.init(hardwareMap);
    }

    public void update() {
        claw.update();
        transfer.update();
    }
}
