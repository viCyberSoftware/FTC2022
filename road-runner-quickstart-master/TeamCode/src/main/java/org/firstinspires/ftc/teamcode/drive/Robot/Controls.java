package org.firstinspires.ftc.teamcode.drive.Robot;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.Robot.Robot;

public class Controls {

    private Gamepad gamepad1 = null;
    private Gamepad gamepad2 = null;
    boolean crossIsPressed = false;
    boolean circleIsPressed = false;
    boolean triangleIsPressed = false;
    boolean squareIsPressed = false;
    boolean rightBumperIsPressed = false;
    boolean leftBumperIsPressed = false;
    boolean dpadLeftIsPressed = false;
    boolean dpadRightIsPressed = false;
    boolean dpadUpIsPressed = false;
    boolean dpadDownIsPressed = false;
    ///testing
    public int transferPos = 0;
    boolean ok1=false;
    boolean ok2 = false;
    boolean ok3 = false;
    boolean ok4 = false;
    ////

    public void control(Robot robot){
        /// de schimbat cu gamepad2 !!!!!!

        if(gamepad2.cross){
            if(!crossIsPressed) {
                crossIsPressed = true;
                robot.linearSlide.moveTo(robot.linearSlide.state.SLIDER_GROUND);
            }
        }else{
            crossIsPressed = false;
        }
        if(gamepad2.circle) {
            if (!circleIsPressed) {
                robot.linearSlide.moveTo(robot.linearSlide.state.SLIDER_LOW);
                circleIsPressed = true;
            }
        }
        else{
            circleIsPressed = false;
        }
        if(gamepad2.triangle) {
            if (!triangleIsPressed) {
                robot.linearSlide.moveTo(robot.linearSlide.state.SLIDER_MIDDLE);
                triangleIsPressed = true;
            }
        }
        else{
            triangleIsPressed = false;
        }
        if(gamepad2.square) {
            if (!squareIsPressed) {
                robot.linearSlide.moveTo(robot.linearSlide.state.SLIDER_HIGH);
                squareIsPressed = true;
            }
        }
        else{
            squareIsPressed = false;
        }
        if(gamepad2.right_trigger>0.5){
            robot.linearSlide.moveUp(20);/// de vazut
        }

        if(gamepad2.left_trigger>0.5){
            robot.linearSlide.moveDown(20);  /// de vazut
        }

        if(gamepad2.left_bumper){
            if(!leftBumperIsPressed){
                robot.claw.open();
                leftBumperIsPressed = true;
            }
        }
        else{
            leftBumperIsPressed = false;
        }

        if(gamepad2.right_bumper){
            if(!rightBumperIsPressed){
                robot.claw.close();
                rightBumperIsPressed = true;
            }
        }
        else{
            rightBumperIsPressed = false;
        }

        if(gamepad2.dpad_left) {
            if (!dpadLeftIsPressed) {
                robot.claw.rotateUp();
                dpadLeftIsPressed = true;
            }
        }else{
            dpadLeftIsPressed = false;
        }

        if(gamepad2.dpad_right) {
            if (!dpadRightIsPressed) {
                robot.claw.rotateDown();
                dpadRightIsPressed = true;
            }
        }else{
            dpadRightIsPressed = false;
        }

        if (gamepad2.dpad_up) {
            if (!dpadUpIsPressed && robot.claw.canTransfer()) {
                robot.transfer.moveBack();
                dpadUpIsPressed = true;
            }
        } else {
            dpadUpIsPressed = false;
        }

        if (gamepad2.dpad_down) {
            if (!dpadDownIsPressed && robot.claw.canTransfer()) {
                robot.transfer.moveFront();
                dpadDownIsPressed = true;
            }
        } else {
            dpadDownIsPressed = false;
        }
        ////testing
        if(gamepad1.dpad_down){
            if(!ok1){
                ok1 = true;
                robot.transfer.moveBackTesting();
            }
        }
        else{
            ok1 = false;
        }

        if(gamepad1.dpad_up){
            if(!ok2){
                ok2 = true;
                robot.transfer.moveFrontTesting();
            }
        }
        else{
            ok2 = false;
        }
        if(gamepad1.dpad_right){
            if(!ok3){
                robot.claw.openTesting();
                ok3 = true;
            }
        } else {
            ok3 = false;
        }

        if(gamepad1.dpad_left){
            if(!ok4){
                robot.claw.closeTesting();
                ok4 = true;
            }
        }
        else{
            ok4 = false;
        }
//////
        robot.drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );
        robot.drive.update();

        robot.update();
    }
    public void init(Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
}
