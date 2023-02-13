package org.firstinspires.ftc.teamcode.drive.opmode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.Robot.Claw;
import org.firstinspires.ftc.teamcode.drive.Robot.LinearSlide;
import org.firstinspires.ftc.teamcode.drive.Robot.OpenCV.Camera;
import org.firstinspires.ftc.teamcode.drive.Robot.Robot;
import org.firstinspires.ftc.teamcode.drive.Robot.Transfer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="AutoLeft",group = "drive")

public class AutoRight extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Camera camera;
    public  Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        camera = new Camera();
        camera.init(hardwareMap);
        robot = new Robot();
        robot.init(hardwareMap,gamepad1,gamepad2);
        waitForStart();
        telemetry.addData("id",camera.getId());
        telemetry.update();
        moveToPark(camera.getId());
        //robot.transfer.moveBack();

    }
    public void moveToPark(int position){
        robot.drive.setPoseEstimate(new Pose2d(-31.81, -63.19, Math.toRadians(90.00)));
        if(position == 5){
            TrajectorySequence parkingOne = robot.drive.trajectorySequenceBuilder(new Pose2d(-31.64, -63.19, Math.toRadians(90.00)))
                    .lineTo(new Vector2d(-36.40, -60.19))
                    .lineTo(new Vector2d(-34.99, -48.73))
                    .splineTo(new Vector2d(-36.04, -36.22), Math.toRadians(87.47))
                    .lineTo(new Vector2d(-48.56, -34.46))
                    .lineTo(new Vector2d(-61.78, -33.93))
                    .splineTo(new Vector2d(-60.72, -26.53), Math.toRadians(85.17))
                    .build();

            robot.drive.followTrajectorySequence(parkingOne);
        }
        else if(position == 6){
            TrajectorySequence parkingTwo = robot.drive.trajectorySequenceBuilder(new Pose2d(-31.81, -63.19, Math.toRadians(90.00)))
                    .lineTo(new Vector2d(-37.10, -60.19))
                    .splineTo(new Vector2d(-36.04, -36.22), Math.toRadians(87.47))
                    .lineTo(new Vector2d(-12.07, -35.52))
                    .lineTo(new Vector2d(-11.72, -17.89))
                    .build();

            robot.drive.followTrajectorySequence(parkingTwo);
        }
        else{
            TrajectorySequence parkingThree = robot.drive.trajectorySequenceBuilder(new Pose2d(-31.81, -63.19, Math.toRadians(90.00)))
                    .lineTo(new Vector2d(-37.10, -60.19))
                    .splineTo(new Vector2d(-36.04, -36.22), Math.toRadians(87.47))
                    .lineTo(new Vector2d(-12.07, -35.52))
                    .lineTo(new Vector2d(-11.72, -17.89))
                    .build();

            robot.drive.followTrajectorySequence(parkingThree);
        }

    }

}
