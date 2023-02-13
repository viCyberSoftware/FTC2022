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

@Autonomous(name="AutoRight",group = "drive")

public class AutoLeft extends LinearOpMode {

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
        camera.init(hardwareMap);
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("id", camera.getId());
            telemetry.update();
        }
        //moveToPark(camera.getId());

    }
    public void moveToPark(int position){
        robot.drive.setPoseEstimate(new Pose2d(40.10, -63.19, Math.toRadians(90.00)));

        if(position == 5){
            TrajectorySequence parkingOne = robot.drive.trajectorySequenceBuilder(new Pose2d(40.10, -63.19, Math.toRadians(90.00)))
                    .lineTo(new Vector2d(35.52, -60.54))
                    .lineTo(new Vector2d(35.87, -36.40))
                    .lineTo(new Vector2d(12.60, -35.87))
                    .lineTo(new Vector2d(11.54, -21.24))
                    .build();

            robot.drive.followTrajectorySequence(parkingOne);
        }
        else if(position == 6){
            TrajectorySequence parkingTwo = robot.drive.trajectorySequenceBuilder(new Pose2d(40.10, -63.19, Math.toRadians(90.00)))
                    .lineTo(new Vector2d(35.52, -60.54))
                    .lineTo(new Vector2d(35.87, -36.40))
                    .lineTo(new Vector2d(35.34, -22.47))
                    .build();
            robot.drive.followTrajectorySequence(parkingTwo);
        }
        else{
            TrajectorySequence parkingThree = robot.drive.trajectorySequenceBuilder(new Pose2d(40.10, -63.19, Math.toRadians(90.00)))
                    .lineTo(new Vector2d(35.52, -60.54))
                    .lineTo(new Vector2d(35.87, -35.34))
                    .lineTo(new Vector2d(59.49, -34.99))
                    .lineToSplineHeading(new Pose2d(59.31, -25.12, Math.toRadians(90.00)))
                    .build();

            robot.drive.followTrajectorySequence(parkingThree);
        }
    }

}
