package org.firstinspires.ftc.teamcode.AUTONOMIE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.elicopter.surub;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.elicopter.slide;
import org.firstinspires.ftc.teamcode.recunoștință.Detection;

@Config
@Autonomous(group = "autonomie")

public class teste extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        slide slide = new slide();
        surub servouri = new surub();
        servouri.autoINIT(hardwareMap);
        slide.sliderINIT(hardwareMap);
        Detection detection = new Detection();
        detection.VisionInitialization(hardwareMap, telemetry);
        Pose2d start = new Pose2d(0,0, Math.toRadians(90));
        drive.setPoseEstimate(start);

        double slowerVelocity = 30;
        double slowerAcceleration = 15;

        //TODO TRAIECTORII

        //preload si aliniere cu stiva

        Trajectory test = drive.trajectoryBuilder(start)
//                .lineTo(
//                        new Vector2d(0, 70),
//                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
//                )
                .lineToConstantHeading(new Vector2d(0,70))
                .build();

        Trajectory spate = drive.trajectoryBuilder(test.end())
                .back(70)
                .build();

        //TODO START
        waitForStart();

      detection.detectare(telemetry);

        if(opModeIsActive()){
            drive.followTrajectory(test);
            drive.followTrajectory(spate);
            sleep(500);
        }}

    //TODO FUNCTII:

}