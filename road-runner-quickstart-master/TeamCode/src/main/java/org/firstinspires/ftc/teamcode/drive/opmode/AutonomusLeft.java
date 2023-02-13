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

public class AutonomusLeft extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Camera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        camera = new Camera();
        camera.init(hardwareMap);
        Robot robot = new Robot();
        robot.init(hardwareMap,gamepad1,gamepad2);
        waitForStart();
        //robot.transfer.moveBack();
        //robot.linearSlide.moveTo(LinearSlide.State.SLIDER_HIGH);
        robot.drive.setPoseEstimate(new Pose2d(13.31, -58.60, Math.toRadians(90.00)));
        TrajectorySequence untitled2 = robot.drive.trajectorySequenceBuilder(new Pose2d(13.31, -58.60, Math.toRadians(90.00)))
                .lineTo(new Vector2d(13.66, -41.33))
                .build();
        robot.drive.followTrajectorySequence(untitled2);
      //  while (opModeIsActive() && !isStopRequested()) {
//
     //   }
        //return;

/*        TrajectorySequence mergLaStalp = robot.drive.trajectorySequenceBuilder(new Pose2d(31.68, -63.50, Math.toRadians(90.00)))
                .lineTo(new Vector2d(12.00, -63.49))
                .lineTo(new Vector2d(12.00, -36.00))
                .splineTo(new Vector2d(5.90, -29.17), Math.toRadians(139.40))
                .build();
        robot.drive.followTrajectorySequence(mergLaStalp);

        while (opModeIsActive() && !isStopRequested() && robot.linearSlide.state!=LinearSlide.State.SLIDER_HIGH) {
            //telemetry.addData("lolMuie",1);
           // telemetry.update();
            telemetry.addData("lolMuie",camera.getId());
            ////iancu talent cu primul co
            //n


            //if camera.id != -1
            //faci pe cazuri unde sa se duca pentru cele 3 id-uri
            //in ultimele 10 secunde porneste spre destinatie

            telemetry.update();
        }

        robot.transfer.moveFront();
        while(opModeIsActive() && !isStopRequested() && robot.transfer.state != Transfer.State.TRANSFER_FRONT){
            telemetry.addData("lolMuie",camera.getId());

        }
        robot.claw.open();
        while(robot.claw.openState!= Claw.State.CLAW_OPEN) {
            telemetry.addData("lolMuie",camera.getId());

        }
        TrajectorySequence maRetrag = robot.drive.trajectorySequenceBuilder(new Pose2d(5.90, -29.17, Math.toRadians(139.40)))
                .splineToSplineHeading(new Pose2d(12.00, -36.00, Math.toRadians(90.00)), Math.toRadians(-45.00))
                .lineTo(new Vector2d(12.00, -12.00))
                .build();
        robot.drive.followTrajectorySequence(maRetrag);
        moveToPark(camera.getId());
        */

    }
    public void moveToPark(int position){
        if(position == 5){
            ///
        }
        else if(position == 6){
            //
        }
        else{
            ///
        }
    }

}
