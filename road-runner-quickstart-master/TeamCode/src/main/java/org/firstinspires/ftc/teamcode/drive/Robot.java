package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    public static class LinearSlide {
        public enum State {
            SLIDER_GROUND(initialPosition),
            SLIDER_LOW(750),
            SLIDER_MIDDLE(1500),
            SLIDER_HIGH(2200);

            private int position;

            private State(int position) {
                this.position = position;
            }

            public int getPosition() {
                return position;
            }
        }

        public enum Stage {
            SLIDER_MOVING_UP,
            SLIDER_MOVING_DOWN,
            SLIDER_STATIONARY
        }

        public static DcMotorEx motorSlideLeft = null;
        public static DcMotorEx motorSlideRight = null;
        public static int initialPosition = 0;

        public State state = null;
        public Stage stage = null;

        public void init(HardwareMap hardwareMap) {
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

            state = State.SLIDER_GROUND;
            stage = Stage.SLIDER_STATIONARY;

        }

        public void setTargetPosition(int position) {
            motorSlideLeft.setTargetPosition(position);
            motorSlideRight.setTargetPosition(position);
        }

        private void setMotorsPower(double power) {
            motorSlideLeft.setPower(power);
            motorSlideRight.setPower(power);
        }

        private boolean motorsAtTarget() {
            return (motorSlideRight.getCurrentPosition() == motorSlideRight.getTargetPosition() && motorSlideLeft.getCurrentPosition() == motorSlideLeft.getTargetPosition());
        }

        private void motorsOn() {
            setMotorsPower(0.5);
        }

        private void motorsOff() {
            setMotorsPower(0);
        }

        private boolean slidersMoving() {
            return (stage == Stage.SLIDER_MOVING_UP || stage == Stage.SLIDER_MOVING_DOWN);
        }

        public void moveUp() {
            switch (state) {
                case SLIDER_GROUND:
                    state = State.SLIDER_LOW;
                    break;

                case SLIDER_LOW:
                    state = State.SLIDER_MIDDLE;
                    break;

                case SLIDER_MIDDLE:
                    state = State.SLIDER_HIGH;
                    break;
            }
            stage = Stage.SLIDER_MOVING_UP;
            setTargetPosition(state.getPosition());
            motorsOn();
        }

        public void moveDown() {
            switch (state) {
                case SLIDER_HIGH:
                    state = State.SLIDER_MIDDLE;
                    break;

                case SLIDER_MIDDLE:
                    state = State.SLIDER_LOW;
                    break;

                case SLIDER_LOW:
                    state = State.SLIDER_GROUND;
            }
            stage = Stage.SLIDER_MOVING_DOWN;
            setTargetPosition(state.getPosition());
            motorsOn();
        }

        public void update() {
            if (slidersMoving() && motorsAtTarget()) {
                stage = Stage.SLIDER_STATIONARY;
                if(state == State.SLIDER_GROUND) motorsOff();
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
        }

        ;

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
