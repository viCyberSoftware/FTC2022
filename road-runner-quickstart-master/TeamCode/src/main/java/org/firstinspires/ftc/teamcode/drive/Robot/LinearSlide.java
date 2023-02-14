package org.firstinspires.ftc.teamcode.drive.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearSlide {
    public enum State {
        SLIDER_GROUND(initialPosition + 0),
        SLIDER_LOW(initialPosition + 750),
        SLIDER_MIDDLE(initialPosition +1500),
        SLIDER_HIGH(initialPosition + 2200),
        SLIDER_FREE_CONTROL(initialPosition + 0) ;

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

    public DcMotorEx motorSlideLeft = null;
    public DcMotorEx motorSlideRight = null;

    public State state = null;
    public Stage stage = null;

    public static int initialPosition = 0;

    public void init(HardwareMap hardwareMap) {
        motorSlideLeft = hardwareMap.get(DcMotorEx.class, "motorSlideLeft");
        motorSlideRight = hardwareMap.get(DcMotorEx.class, "motorSlideRight");

        motorSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //initialPosition = motorSlideLeft.getCurrentPosition();
        motorSlideRight.setTargetPosition(0);
        motorSlideLeft.setTargetPosition(0);

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

    public boolean motorsAtTarget() { //MODIFICAT ATENTIE APROXIMATIV
        return (Math.abs(motorSlideRight.getCurrentPosition() - motorSlideRight.getTargetPosition()) <= 30);
        //return (motorSlideRight.getCurrentPosition() == motorSlideRight.getTargetPosition() || motorSlideLeft.getCurrentPosition() == motorSlideLeft.getTargetPosition());
    }

    private void motorsOn() {
        setMotorsPower(0.8);
    }

    private void motorsOff() {
        setMotorsPower(0);
    }

    public boolean slidersMoving() {
        return (stage == Stage.SLIDER_MOVING_UP || stage == Stage.SLIDER_MOVING_DOWN);
    }

    public void moveTo(State newState){
        if(this.state.position < newState.position){
            this.stage = Stage.SLIDER_MOVING_UP;
        }
        else if(this.state.position > newState.position){
            this.stage = Stage.SLIDER_MOVING_DOWN;
        }
       this.state = newState;
        motorsOn();
        setTargetPosition(state.getPosition());
    }

    public void moveUp(int distance) {
        this.state = State.SLIDER_FREE_CONTROL;
        stage = Stage.SLIDER_MOVING_UP;
        int newPosition = motorSlideLeft.getCurrentPosition() + distance;// de vazut
        if (newPosition <= State.SLIDER_HIGH.position) {
            setTargetPosition(newPosition);
            motorsOn();
        }
    }

    public void moveDown(int distance) {
        this.state = State.SLIDER_FREE_CONTROL;
        stage = Stage.SLIDER_MOVING_DOWN;
        int newPosition = motorSlideLeft.getCurrentPosition() - distance;// de vazut
        if(newPosition>=0) {
            setTargetPosition(newPosition);
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
