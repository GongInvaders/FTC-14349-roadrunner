package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.PIDCoefficients;


import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;




@TeleOp(name="Primo Drive Epic (not clickbait)")

public class PrimoDriveEpic extends LinearOpMode {
    
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    

    // Declare OpMode members.
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
    private Servo GoatedGrabber = null;


    private ColorSensor Colour_REV_ColorRangeSensor = null;
    private RevBlinkinLedDriver ledStrip = null;

    private TouchSensor button = null;

    double driveadjust = 2;


    

    @Override
    public void runOpMode() {
        double ServoPosition;
        double ServoSpeed;

        ServoPosition = 0.7;
        ServoSpeed = 0.01;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int gain;
        NormalizedRGBA normalizedColours;
        int color;
        float hue;
        float saturation;
        float value;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        rightfront  = hardwareMap.get(DcMotor.class, "Rightfront");
        rightback = hardwareMap.get(DcMotor.class, "Rightback");
        leftfront = hardwareMap.get(DcMotor.class, "Leftfront");
        leftback = hardwareMap.get(DcMotor.class, "Leftback");
        leftfront.setDirection(DcMotor.Direction.REVERSE);
        leftback.setDirection(DcMotor.Direction.REVERSE);
        
        Spool = hardwareMap.get(DcMotor.class, "Spool");
        GrabSpin = hardwareMap.get(DcMotor.class, "GrabSpin");
        Carousel = hardwareMap.get(DcMotor.class, "Carousel");
        MagnetLift = hardwareMap.get(DcMotor.class, "MagnetLift");
        CapArm = hardwareMap.get(Servo.class, "CapArm");
        CapGrab = hardwareMap.get(Servo.class, "CapGrab");
        Flap = hardwareMap.get(Servo.class,"Flap");
        GoatedGrabber = hardwareMap.get(Servo.class,"GoatedGrabber");

        button = hardwareMap.get(TouchSensor.class, "touch");

        double throttle = 1.0;

        Colour_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "Colour");
        ledStrip = hardwareMap.get(RevBlinkinLedDriver.class, "LEDStrip");

        Spool.setDirection(DcMotor.Direction.REVERSE);



        Spool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //GrabSpin.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator(); CANT FIND ERROR SHOWS UP

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        gain = 2;
        
        // wobble.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //  wobble.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        
        boolean toggle50 = false;
        boolean toggle25 = false;
        
        //Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Spool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MagnetLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CapArm.setPosition(0.5);

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        //LEFT MOTORS ARE REVERSED THEREFORE NEED TO BE SET TO NEGATIVE VALUES TO GO FORWARD
        while (opModeIsActive()) {
            ((NormalizedColorSensor) Colour_REV_ColorRangeSensor).setGain(gain);
            normalizedColours = ((NormalizedColorSensor) Colour_REV_ColorRangeSensor).getNormalizedColors();
            color = normalizedColours.toColor();
            hue = JavaUtil.colorToHue(color);
            saturation = JavaUtil.colorToSaturation(color);
            value = JavaUtil.colorToValue(color);

            if(((DistanceSensor) Colour_REV_ColorRangeSensor).getDistance(DistanceUnit.CM) < 3){
                if (hue < 90) {
                    telemetry.addData("Object", "Cube");
                    ledStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
                } else if (hue < 150) {
                    telemetry.addData("Object", "Duck");
                    ledStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
                } else if (hue < 225) {
                    telemetry.addData("Object", "Ball");
                    ledStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
                }
            } else {
                ledStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }

            // rightshoot.setVelocityPIDFCoefficients(125,0,70,13.5);
            double G1rightstickY = gamepad1.right_stick_y;
            double G1leftstickY = gamepad1.left_stick_y;
            double G1rightstickX = gamepad1.right_stick_x;
            double G1leftstickX = gamepad1.left_stick_x;
            double G1lefttrigger = gamepad1.left_trigger * 0.6;
            double G1righttrigger = gamepad1.right_trigger * 0.6;

            if (gamepad1.a) {
                driveadjust = driveadjust + 1;
                sleep(200);
            }


            if ((driveadjust % 2) != 0) {
                // ledservo.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                if (G1lefttrigger >= 0.1) {
                    rightfront.setPower(G1lefttrigger * 0.6);
                    rightback.setPower(-G1lefttrigger * 0.6);
                    leftfront.setPower(G1lefttrigger * 0.6);
                    leftback.setPower(-G1lefttrigger * 0.6);
                } else if (G1righttrigger >= 0.1) {
                    rightfront.setPower(-G1righttrigger * 0.6);
                    rightback.setPower(G1righttrigger * 0.6);
                    leftfront.setPower(-G1righttrigger * 0.6);
                    leftback.setPower(G1righttrigger * 0.6);
                } else {
                    rightfront.setPower(-G1rightstickY * 0.6);
                    rightback.setPower(-G1rightstickY * 0.6);
                    leftfront.setPower(G1leftstickY * 0.6);
                    leftback.setPower(G1leftstickY * 0.6);

                }


            } else {
                //ledservo.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

                if (G1lefttrigger >= 0.1) {
                    rightfront.setPower(G1lefttrigger);
                    rightback.setPower(-G1lefttrigger);
                    leftfront.setPower(G1lefttrigger);
                    leftback.setPower(-G1lefttrigger);
                } else if (G1righttrigger >= 0.1) {
                    rightfront.setPower(-G1righttrigger);
                    rightback.setPower(G1righttrigger);
                    leftfront.setPower(-G1righttrigger);
                    leftback.setPower(G1righttrigger);
                } else {
                    rightfront.setPower(-G1rightstickY);
                    rightback.setPower(-G1rightstickY);
                    leftfront.setPower(G1leftstickY);
                    leftback.setPower(G1leftstickY);

                }

            }
        if(gamepad2.right_stick_y>=0.1 && !button.isPressed()){
            //surgical.setPower(-1);
            Spool.setPower(gamepad2.right_stick_y);
            
        } else if(gamepad2.right_stick_y<=-0.1){
            //surgical.setPower(1);
            Spool.setPower(gamepad2.right_stick_y);
        } else {
            //surgical.setPower(0);
            Spool.setPower(0);
        }
        if (gamepad1.x){
            throttle = 2;
        } else{
            throttle = 1;
        }
        if(gamepad2.left_stick_y>=0.1){
            //surgical.setPower(-1);
            MagnetLift.setPower(-1*gamepad2.left_stick_y);

        } else if(gamepad2.left_stick_y<=-0.1) {
            //surgical.setPower(1);
            MagnetLift.setPower(-1 * gamepad2.left_stick_y);
        }else if(gamepad2.a){
                MagnetLift.setPower(-0.2);
            }
        else {
            //surgical.setPower(0);
            MagnetLift.setPower(0);
        }

        
        
        if(gamepad1.left_bumper){
            Carousel.setPower(0.5*throttle);
        }
        else if(gamepad1.right_bumper){
            Carousel.setPower(-0.5 *throttle);
        }
        else{
            Carousel.setPower(0);
        }
        
//wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            if(gamepad2.right_trigger>=0.1){
                Flap.setPosition(0.75);
                
                //Spool.setPower(-0.8*gamepad2.right_trigger);
            
                
            }
            else{
                Flap.setPosition(0.95);
               
            }
            
            
            if ((gamepad2.dpad_up) && (ServoPosition < 0.75)){
                ServoPosition += ServoSpeed;
            }
            
            else if ((gamepad2.dpad_down) && (ServoPosition > 0)){
                ServoPosition += -ServoSpeed;
            }
            CapArm.setPosition(ServoPosition);


            
            if (gamepad2.left_trigger>=0.1){
                CapGrab.setPosition(1);
            }
            
            else {
                CapGrab.setPosition(0.63);
            }
/*
            if (gamepad2.a && 1135 <= (rightshoot.getVelocity()/(103.6/60)) && (rightshoot.getVelocity()/(103.6/60))<= 1170){
                ledservo.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                sleep(50);
                
            } // pressing a button
*/
            if (gamepad2.right_bumper){
            //ledservo.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                //runUsingVelocity(rightshoot, 1100, 103.6);
                GrabSpin.setPower(1);
                
            }
            else if (gamepad2.left_bumper){
                //runUsingVelocity(rightshoot, 1200, 103.6);
                //ledservo.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                GrabSpin.setPower(-1);
                
            }
            else{
                //rightshoot.setPower(0);
                //button=0;
           // ledservo.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                GrabSpin.setPower(0);
            }
            

      
            
                telemetry.addData("driveadjust", driveadjust);                
                telemetry.addData("leftfront", leftfront.getCurrentPosition());
                telemetry.addData("leftback", leftback.getCurrentPosition());
                telemetry.addData("rightfront", rightfront.getCurrentPosition());
                telemetry.addData("rightback", rightback.getCurrentPosition());
                telemetry.addData("Arm Position: ", Spool.getCurrentPosition());
                telemetry.addData("Servo Position: ", ServoPosition);
                telemetry.addData("Gamepad 1 A: ", gamepad1.a);
                telemetry.addData("Gamepad 1 B: ", gamepad1.b);

                telemetry.addData("left-trigger: ", G1lefttrigger);
                telemetry.addData("right-trigger: ", G1righttrigger);
                telemetry.addData("button: ", button.getValue());
               // telemetry.addData("RealVelocity: ", rightshoot.getVelocity()/(103.6/60));

               // telemetry.addData("toggle", toggle);
               // telemetry.addData("Flap", Flap.getPosition());

                telemetry.update();
                
                

        

                
  
  

            
        
        }
    }
        public void runUsingVelocity(DcMotorEx motor, double rpm, double ticksPerRev){
        //motor.setVelocity(rpm*ticksPerRev/60);
    }

    
    
}


