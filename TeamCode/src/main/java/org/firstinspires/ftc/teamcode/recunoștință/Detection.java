package org.firstinspires.ftc.teamcode.recunoștință;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Detection {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag IDs 4, 20 ans 24 from the 36h11 family
    public int Caz1 = 4;
    public int Caz2 = 20;
    public int Caz3 = 24;

    public AprilTagDetection tagOfInterest = null;

    public void VisionInitialization (HardwareMap hardwareMap, Telemetry telemetry){

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
    }

    public void detectare (Telemetry telemetry) throws InterruptedException{

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0)
        {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == Caz1 || tag.id == Caz2 || tag.id == Caz3)
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if(tagFound)
            {
                telemetry.addLine("Tag of interest is in sight!");
                telemetry.addLine(String.format("\nDetected tag ID=%d", tagOfInterest.id));
            }
            else
            {
                telemetry.addLine("Don't see tag of interest ");
            }
        }
        else
        {
            telemetry.addLine("Don't see tag of interest. ");

        }
        telemetry.update();

        if (tagOfInterest.id == Caz1)
            sleep(20);
    }

}
