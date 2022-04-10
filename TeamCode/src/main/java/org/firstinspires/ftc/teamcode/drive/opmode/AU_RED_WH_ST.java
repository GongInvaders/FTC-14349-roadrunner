package org.firstinspires.ftc.teamcode.drive.opmode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.time.Duration;
import java.time.Instant;


/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class AU_RED_WH_ST extends LinearOpMode {

    private OpenCvCamera webcam;
    private ContourPipeline pipeline;

    private double crThreshHigh = 150;
    private double crThreshLow = 120;
    private double cbThreshHigh = 255;
    private double cbThreshLow = 255;

    private int minRectangleArea = 1000;
    private double leftBarcodeRangeBoundary = 0.3; //i.e 30% of the way across the frame from the left
    private double rightBarcodeRangeBoundary = 0.6; //i.e 60% of the way across the frame from the left

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
    private DcMotor MagnetLift = null;
    private Servo Flap = null;
    private Servo CapArm = null;
    private Servo CapGrab = null;

    double driveadjust = 2;

    boolean right = false;
    boolean centre = false;
    boolean left = false;
    private ColorSensor Colour_REV_ColorRangeSensor;





    @RequiresApi(api = Build.VERSION_CODES.O)
    @Override
    public void runOpMode() throws InterruptedException {


        Spool = hardwareMap.get(DcMotor.class, "Spool"); //Ticks per revolution of this motor = 140
        GrabSpin = hardwareMap.get(DcMotor.class, "GrabSpin");
        Carousel = hardwareMap.get(DcMotor.class, "Carousel"); //Ticks per revolution is 56
        MagnetLift = hardwareMap.get(DcMotor.class, "MagnetLift");
        CapArm = hardwareMap.get(Servo .class, "CapArm");
        CapGrab = hardwareMap.get(Servo.class, "CapGrab");
        Flap = hardwareMap.get(Servo.class,"Flap");



        Spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        GrabSpin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int gain = 2;
        NormalizedRGBA normalizedColours;
        int color;
        float hue;
        float saturation;
        float value;


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

        pipeline = new ContourPipeline(0, 0, 0, 0);

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
        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(webcam, 20);

        sleep(3000);



        waitForStart();
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
        pipeline = null;

        if (isStopRequested()) return;



        Trajectory hub = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-10, -42), Math.toRadians(90))
                .build();
        Trajectory safetyback = drive.trajectoryBuilder(hub.end())
                .back(15)
                .build();

        Trajectory lineup = drive.trajectoryBuilder(safetyback.end())
                .lineToLinearHeading(new Pose2d(12.7, -65, 0))
                .addTemporalMarker(0, ()->{
                    CapArm.setPosition(0.7);
                    Spool.setTargetPosition(200);
                    Spool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Spool.setPower(1);})
                .build();

        Trajectory warehouse = drive.trajectoryBuilder(lineup.end())
                .lineToLinearHeading(new Pose2d(38.7, -64, 0))
                .build();
        Trajectory park = drive.trajectoryBuilder(warehouse.end())
                .strafeLeft(20)
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
        CapGrab.setPosition(0.72);
        CapArm.setPosition(0.25);
        if (right == true){
            Spool.setTargetPosition(1590);
            //CapArm.setPosition(0.25);

        } else if (left == true){
            Spool.setTargetPosition(0);
            //CapArm.setPosition(0.25);
        } else {
            Spool.setTargetPosition(935);
            //CapArm.setPosition(0.25);
        }

        Spool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Spool.setPower(1);
        drive.followTrajectory(hub);
        while (Spool.isBusy()){

        }
        Spool.setPower(0);
        CapGrab.setPosition(1);
        sleep(250);
        drive.followTrajectory(safetyback);


        drive.followTrajectory(lineup);
        while (Spool.isBusy()){

        }
        Spool.setPower(0);

        /*Instant detectLength = Instant.now();
        while (!(((DistanceSensor) Colour_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) < 3.0)){

        }*/
        //drive.breakFollowing();
        drive.followTrajectory(warehouse);
        drive.followTrajectory(park);



    }
}
