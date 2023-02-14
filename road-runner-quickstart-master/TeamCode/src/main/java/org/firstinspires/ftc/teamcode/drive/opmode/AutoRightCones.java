package org.firstinspires.ftc.teamcode.drive.opmode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robot.Claw;
import org.firstinspires.ftc.teamcode.drive.Robot.OpenCV.Camera;
import org.firstinspires.ftc.teamcode.drive.Robot.Robot;
import org.firstinspires.ftc.teamcode.drive.Robot.Transfer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="AutoRightCones",group = "drive")

public class AutoRightCones extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Camera camera;
    public  Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        camera = new Camera();
        robot = new Robot();
        robot.init(hardwareMap, null, null, telemetry);

        waitForStart();

        //robot.transfer.movePark();

        //camera.init(hardwareMap);

        //Set trajectories
        TrajectorySequence AutoRightHighJunction = robot.drive.trajectorySequenceBuilder(new Pose2d(40.10, -61.62, Math.toRadians(-90.00)))
                .lineToSplineHeading(new Pose2d(12.50, -60.00, Math.toRadians(-90.00)))
                .lineTo(new Vector2d(12.77, -38.51))
                .lineToLinearHeading(new Pose2d(9.18, -33, Math.toRadians(323.0)))
                .build();
        TrajectorySequence AutoRightRetreat = robot.drive.trajectorySequenceBuilder(new Pose2d(9.18, -33, Math.toRadians(323.00)))
                .lineToSplineHeading(new Pose2d(12.00, -12.00, Math.toRadians(-90.00)))
                .build();
        TrajectorySequence AutoRightPark2 = robot.drive.trajectorySequenceBuilder(new Pose2d(12.00, -12.00, Math.toRadians(-90.00)))
                .lineTo(new Vector2d(34.11, -12.00))
                .build();
        TrajectorySequence AutoRightPark3 = robot.drive.trajectorySequenceBuilder(new Pose2d(12.00, -12.00, Math.toRadians(-90.00)))
                .lineTo(new Vector2d(60.00, -12.00))
                .build();




        //Set starting position
        robot.drive.setPoseEstimate(new Pose2d(40.10, -61.62, Math.toRadians(-90.00)));

        //Drive to high junction
        robot.drive.followTrajectorySequence(AutoRightHighJunction);

        //Update (refresh states)
        robot.update();

        //Raise slides
        robot.raiseToHighJunction();
        do {
            robot.update();
            if (robot.claw.rotationState == Claw.State.CLAW_DOWN) {
                telemetry.addLine("claw down");
            }
            if (robot.linearSlide.motorsAtTarget()) {
                telemetry.addLine("motors at target");
            }
            if (robot.transfer.state == Transfer.State.TRANSFER_BACK) {
                telemetry.addLine("transfer back");
            }
            if (robot.transfer.state == Transfer.State.TRANSFER_MOVING_BACK) {
                telemetry.addLine("transfer moving back");
            }
            if (robot.transfer.state == Transfer.State.UNDEFINED) {
                telemetry.addLine("undefined");
            }
            telemetry.update();
        } while (opModeIsActive() && !isStopRequested() && robot.action != Robot.Action.IDLE);

        //Update (refresh states)
        robot.update();

        //Drop the cone and wait a bit
        robot.claw.open();
        while (opModeIsActive() && !isStopRequested()) {
            robot.update();
            if (robot.claw.openState == Claw.State.CLAW_OPEN) {
                break;
            }
        }
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && !isStopRequested()) {
            if (timer.seconds() >= 1) {
                break;
            }
        }

        //Update (refresh states)
        robot.update();

        //Lower the slides
        robot.lower();

        //Wait for lower
        while (opModeIsActive() && !isStopRequested()) {
            if (robot.action == Robot.Action.IDLE) {
                break;
            }
        }

        //Retreat
        robot.drive.followTrajectorySequence(AutoRightRetreat);

        //Park according to sleeve orientation
        switch (camera.getId()) {
            case 5:
                break;
            case 6:
                robot.drive.followTrajectorySequence(AutoRightPark2);
                break;
            default:
                robot.drive.followTrajectorySequence(AutoRightPark3);
                break;
        }

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
