package org.firstinspires.ftc.teamcode.drive;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Controls {

    private Gamepad gamepad1 = null;
    private Gamepad gamepad2 = null;

    public void control(Robot robot){
        /// de schimbat cu gamepad2 !!!!!!
        if(gamepad1.triangle){
            robot.linearSlide.moveUp();
        }
        if(gamepad1.cross){
            robot.linearSlide.moveDown();
        }
    }
    public void init(Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }
}
