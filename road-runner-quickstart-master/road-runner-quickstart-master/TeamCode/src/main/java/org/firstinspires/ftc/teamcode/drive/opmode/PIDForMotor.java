package org.firstInspires.ftc.teamcode.dirve.opmode;

import androidx.annotaion.Nullable'

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpmode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspire.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;

@Config // makes it visible on the dahsboard (ftc dashboard - graphs for pc! -> shows errors)
@Autonomous(name = "PID test")
public class PIFForMotor extends LinearOpMode{

    DcMotorEx motor;
    ElapsedTime timer = nre ElapsedTime();

    private double lastError = 0;
    private double integralSum = 0;

    public static double Kp = 0.0;
    public static double Ki = 0.0;
    public static double Kd = 0.0;

    public static double targetPosition = 5000; // constant

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); // TODO: update for the new hardwareMap type of class ??

        TelemetryPacket packet = new TelemetryPacket();

        dashboard.setTelemetryTransmissionInterval(25); // refresh rate for the dashboard

        motor = hardwareMap.get(DcMotorEx.class, "intakesSlides");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart(); // !!!

        int targetPosition = 5000;

        while(opModeIsActive()){
            double power = returnPower(targetPosition, motor.getCurrentPostion());

            packet.put("power", power); // for the dashboard (a kind of telemetry, but for dashboard)
            packet.put("position", motor.getCurrentPosition());
            packet.put("error", lastError);

            motor.setPower(power);

            dashboard.sendTelemetryPacket(packet);

            // drive.goToPostionIntakeSlides(timer, Kp, Ki, Kd, targetPostion);

        }

        // experiments:

        Vector2d = myVector = newVector(10, -5); // just a position
        Pose2d startPose = newPose2d(10, 08, Math.toRadians(90)); // a position, and also a heading

        // !!! "drive" pos is set to (0, 0, 0) -> should update it
        drive.setPoseEstimate(startPose);

        // path = a spline or straight line between two points
        // trajectory = a collection of paths i guess??

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d()) // new Pose2d = (0, 0, 0), can be modified alternativily
                .strafeRight(10)
                .forward(5)
                .build(); // syntax

        drive.followTrajectory(myTrajectory); // quite straightforward

        // !!! this trajectory will fail :)
        // it is not continous (maths => infinite velocity??) -> "throws InterruptedException"

        // first method to solve this problem:
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10);
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d())
                .forward(5)
                .build();

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);

        // ok method, slows between the two paths
        // alternative method: we add a spline(a curve?) between the two paths, keeping it continous

        Trajectory traj = drive.trajecotryBuilder(new Pose2d())
                .splineTo(new Vector2d(x1, y1), heading)
                .splineTo(new Vector2d(x2, y2), heading)
                .build(); // the variables used are not declared, only meant for idea


        // multiple trajectories:
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end()) // notice how it starts from the other one's end
                .splineTo(new Vector2d(5, 6), 0)
                .splineTo(new Vector2d(9, -10), 0)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(5, 6), 0)
                .splineTo(new Vector2d(9, -10), 0)
                .build();

        drive.followTrajectory(traj1);
        robot.dropServo(); // i have no idea?
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

        // reversed motion:
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(), true) // notice the second paramter
                .splineTo(new Vector2d(36, 36), Math.toRadians(0))
                .build();

        // !!! always build the trajectories beforehand (it's slow)

        //turns:
        drive.turn(Math.toRadians(90)); // ez

        // turns and trajectories
        // !!! do not simply put one after another, it will break

        Trajectory traj1 = drive.trajectoryBuilder(startPose, false)
                .forward(10)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end().plus(new Pose2d(0, 0, Math.toRadians(90)))) // we add the second pose so the position is synced
                .strafeLeft(10)
                .build();

        drive.followTrajectory(traj1);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(traj2);

        // we can also slow down:
        drive.trajectoryBuilder(startPose, false)
                .splineTo(
                        new Vector2d(30, 30), 0,
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .splineTo(new Vector2d(40, 40), Math.toRadians(-90))
                .build();

        // lots of functions to make the movement easier: (trajectoryBuilder Function List)
        // TODO: check (Advanced Tips) for easier teleop implementation and more!
    }

    public double returnPower(double reference, double state){ // maths :)
        double error = refernce - state;
        integralSum += error * timer.seconds();
        double derivate = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivate * Kd) + (integralSum * Ki)l
    }
}