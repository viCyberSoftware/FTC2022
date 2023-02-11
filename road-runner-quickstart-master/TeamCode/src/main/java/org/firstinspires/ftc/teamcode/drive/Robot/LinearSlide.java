package org.firstinspires.ftc.teamcode.drive.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearSlide {
    public enum State {
        SLIDER_GROUND(0),
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

    public State state = null;
    public Stage stage = null;

    public void init(HardwareMap hardwareMap) {
        motorSlideLeft = hardwareMap.get(DcMotorEx.class, "motorSlideLeft");
        motorSlideRight = hardwareMap.get(DcMotorEx.class, "motorSlideRight");

        motorSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initialPosition = motorSlideLeft.getCurrentPosition();

        motorSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorsOn();

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
        return (motorSlideRight.getCurrentPosition() == motorSlideRight.getTargetPosition() || motorSlideLeft.getCurrentPosition() == motorSlideLeft.getTargetPosition());
    }

    private void motorsOn() {
        setMotorsPower(0.8);
    }

    private void motorsOff() {
        setMotorsPower(0);
    }

    private boolean slidersMoving() {
        return (stage == Stage.SLIDER_MOVING_UP || stage == Stage.SLIDER_MOVING_DOWN);
    }

    public void moveUp() {
        if(!slidersMoving()){
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
    }

    public void moveDown() {
        if (!slidersMoving()) {
            switch (state) {
                case SLIDER_HIGH:
                    state = State.SLIDER_MIDDLE;
                    break;

                case SLIDER_MIDDLE:
                    state = State.SLIDER_LOW;
                    break;

                case SLIDER_LOW:
                    state = State.SLIDER_GROUND;
                    break;
            }
            stage = Stage.SLIDER_MOVING_DOWN;
            setTargetPosition(state.getPosition());
            motorsOn();
        }
    }

    public void update() {
        if (slidersMoving() && motorsAtTarget()) {
            //  telemetry.addData("","");
            stage = Stage.SLIDER_STATIONARY;
        }
    }

}
