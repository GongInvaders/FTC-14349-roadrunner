package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.index.qual.SameLen;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.opmode.PrimoDriveEpic;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;



/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class strafingTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftfront = null;
    private DcMotor leftback = null;
    private DcMotor rightfront = null;
    private DcMotor rightback = null;
    private DcMotor Spool = null;

    private DcMotor GrabSpin = null;
    private DcMotor Carousel = null;
    private Servo Flap = null;
    private Servo CapArm = null;
    private Servo CapGrab = null;

    double driveadjust = 2;

    @Override
    public void runOpMode() throws InterruptedException {
        rightfront  = hardwareMap.get(DcMotor.class, "Rightfront");
        rightback = hardwareMap.get(DcMotor.class, "Rightback");
        leftfront = hardwareMap.get(DcMotor.class, "Leftfront");
        leftback = hardwareMap.get(DcMotor.class, "Leftback");

        Spool = hardwareMap.get(DcMotor.class, "Spool"); //Ticks per revolution of this motor = 140
        GrabSpin = hardwareMap.get(DcMotor.class, "GrabSpin");
        Carousel = hardwareMap.get(DcMotor.class, "Carousel"); //Ticks per revolution is 56
        CapArm = hardwareMap.get(Servo .class, "CapArm");
        CapGrab = hardwareMap.get(Servo.class, "CapGrab");
        Flap = hardwareMap.get(Servo.class,"Flap");



        Spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        GrabSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        waitForStart();

        if (isStopRequested()) return;

        Trajectory hub = drive.trajectoryBuilder(startPose)
                .strafeLeft(24, SampleMecanumDrive.getVelocityConstraint(5, .5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectory(hub);

    }
}
