package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous(name="Webcam Dashboard Test")
public class WebcamTest extends LinearOpMode {

    // TODO: fill in
    public static final String VUFORIA_LICENSE_KEY = "ATslaan/////AAABmbxBgEJNy0GasYoJa/ZPoJhBRoPfb+AX5bp19iHJBtK2ao5ToApYco1n17TGL0MpeJyIK8TxYc2Mp4cxHNR5ACpgQ2BrjGKdS8x2xR+21jmtqprDKtUa5lPjGRI6y8y49ixnVK/aEgaXe5iTAPFl17OBFfqDpP56ymA0R3xanqF1npvYy4veRkmNSG5LME5tQmJs9dtnfmz3skmpoGONN8Nu+WLJQjxyIigouw1IHO4qOZt1o+X3ViZZ1QEbB4CwB/n3VL8ZfyB4r0J/OCvMBkzUo0s4DDKgJ836LXMaBQZvSczTYpGmmJ9oKWFI6VwUw3CffA5bRPZ0jq4Sky1K7mhtsDxJwLttH0VNgaFxSy+w";

    @Override
    public void runOpMode() throws InterruptedException {
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters();
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        waitForStart();

        while (opModeIsActive());
    }
}