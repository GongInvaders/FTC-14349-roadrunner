package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
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


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {

    private OpenCvCamera webcam;
    private ContourPipeline pipeline;

    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;

    private int minRectangleArea = 2000;
    private double leftBarcodeRangeBoundary = 0.3; //i.e 30% of the way across the frame from the left
    private double rightBarcodeRangeBoundary = 0.7; //i.e 60% of the way across the frame from the left

    private double lowerRuntime = 0;
    private double upperRuntime = 0;

    // Determining Green Range                        Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 120.0);


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

    boolean right = false;
    boolean centre = false;
    boolean left = false;






    @Override
    public void runOpMode() throws InterruptedException {


        Spool = hardwareMap.get(DcMotor.class, "Spool"); //Ticks per revolution of this motor = 140
        GrabSpin = hardwareMap.get(DcMotor.class, "GrabSpin");
        Carousel = hardwareMap.get(DcMotor.class, "Carousel"); //Ticks per revolution is 56
        CapArm = hardwareMap.get(Servo .class, "CapArm");
        CapGrab = hardwareMap.get(Servo.class, "CapGrab");
        Flap = hardwareMap.get(Servo.class,"Flap");



        Spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        GrabSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        /*void startArm(height){
            int bottom = 0;
            int low = 0;
            int mid = 0;
            int top = 2380;
            switch (position)
            Spool.setTargetPosition(positionValue);
            Spool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Spool.setPower(1);

        }

        static void endArm(){
            while (Spool.isBusy()){

            }
            Spool.setPower(0);

        }
*/


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12.7, -62.5, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //OpenCV Pipeline

        pipeline = new ContourPipeline(0.2, 0.2, 0.2, 0.2);

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);

        webcam.setPipeline(pipeline);

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(webcam, 20);

        sleep(3000);

        if(pipeline.getRectArea() > minRectangleArea){
            //Then check the location of the rectangle to see which barcode it is in.
            if(pipeline.getRectMidpointX() > (rightBarcodeRangeBoundary * 640)){
                telemetry.addData("Barcode Position", "Right");
                right = true;
            }
            else if(pipeline.getRectMidpointX() < (leftBarcodeRangeBoundary * 640)){
                telemetry.addData("Barcode Position", "Left");
                left = true;
            }
            else {
                telemetry.addData("Barcode Position", "Centre");
                centre = true;
            }
        }
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        Trajectory hub = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-10, -40), Math.toRadians(90))
                .build();
        Trajectory carousel = drive.trajectoryBuilder(hub.end(), true)
                .splineTo(new Vector2d(-62, -48), Math.toRadians(180))
                .addTemporalMarker(1, ()->{
                    Flap.setPosition(0.95);
                    Spool.setTargetPosition(0);
                    Spool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Spool.setPower(1);})
                .build();
        Trajectory strafe = drive.trajectoryBuilder(carousel.end())
                .lineTo(new Vector2d(-62,-34.5),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory backstrafe = drive.trajectoryBuilder(carousel.end())
                .lineTo(new Vector2d(-64,-32),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory backup = drive.trajectoryBuilder(backstrafe.end())
                .back(4)
                .build();
        Trajectory forwardafterstrafe = drive.trajectoryBuilder(strafe.end())
                .forward(34,
                    SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(34, ()->{
                    Spool.setTargetPosition(140*17);
                    Spool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Spool.setPower(1);


                })
                .build();
        Trajectory duckdrop = drive.trajectoryBuilder(forwardafterstrafe.end().plus(new Pose2d(0, 0, Math.toRadians(15))))
                .splineTo(new Vector2d(-24, -30), Math.toRadians(30))
                .build();

        Trajectory prepareforcarousel = drive.trajectoryBuilder(duckdrop.end(), true)
                .splineTo(new Vector2d(-62,-48), Math.toRadians(180))
                .addTemporalMarker(1, ()->{
                    Spool.setTargetPosition(0);
                    Spool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Spool.setPower(1);})
                .build();
        Trajectory carouselstrafe = drive.trajectoryBuilder(prepareforcarousel.end(), true)
                .lineTo(new Vector2d(-62,-56),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory park = drive.trajectoryBuilder(carouselstrafe.end(), true)
                .lineTo(new Vector2d(-62,-34.5),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        /*Trajectory traj2 = drive.trajectoryBuilder(traj1.end(),true)
                .splineTo(new Vector2d(-20,-56),Math.toRadians(180))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(20,-60),0)
                .forward(24)
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(),true)
                .back(24)
                .splineTo(new Vector2d(-20,-56), Math.toRadians(180))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .splineTo(new Vector2d(-10, -36), Math.toRadians(90))
                .build();*/

        Flap.setPosition(0.95);
        if (right == true){
            Spool.setTargetPosition(140*17);
        } else if (left == true){
            Spool.setTargetPosition(814);
        } else {
            Spool.setTargetPosition(1575);
        }

        Spool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Spool.setPower(1);
        drive.followTrajectory(hub);
        while (Spool.isBusy()){

        }
        Spool.setPower(0);
        sleep(500);
        if(right == true){
            Flap.setPosition(0.75);
            sleep(250);
        } else{
            GrabSpin.setTargetPosition(-600);
            GrabSpin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            GrabSpin.setPower(1);
            while(GrabSpin.isBusy()){

            }
            GrabSpin.setPower(0);
        }
        drive.followTrajectory(carousel);
        while (Spool.isBusy()){

        }
        Spool.setPower(0);
        //drive.followTrajectory(strafe);
        /*Carousel.setTargetPosition(3000);
        Carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Carousel.setPower(0.2);

        while (Carousel.isBusy()){

        }
        Carousel.setPower(0);
*/

        GrabSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);


        GrabSpin.setTargetPosition(1400);
        GrabSpin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.followTrajectory(strafe);
        GrabSpin.setPower(1);
        drive.followTrajectory(forwardafterstrafe);
        while (GrabSpin.isBusy()){

        }
        GrabSpin.setPower(0);
        while (Spool.isBusy()){

        }
        Spool.setPower(0);
        drive.followTrajectory(duckdrop);
        Flap.setPosition(0.75);
        sleep(600);
        Flap.setPosition(0.95);
        drive.followTrajectory(prepareforcarousel);
        while (Spool.isBusy()){

        }
        Spool.setPower(0);
        drive.followTrajectory(carouselstrafe);
        Carousel.setTargetPosition(1000);
        Carousel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Carousel.setPower(0.1);

        while (Carousel.isBusy()){

        }
        Carousel.setPower(0);
        drive.followTrajectory(park);




    }
}
