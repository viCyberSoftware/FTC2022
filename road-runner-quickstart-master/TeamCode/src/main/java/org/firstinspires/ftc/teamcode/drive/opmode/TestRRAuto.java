package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive")
public class TestRRAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(0, 0))
                .splineTo(new Vector2d(47.25, -16.5), Math.toRadians(180))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                drive.followTrajectory(traj1);
            }
        }
    }
}
