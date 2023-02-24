package org.firstinspires.ftc.teamcode.AUTONOMIE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.elicopter.slide;
import org.firstinspires.ftc.teamcode.elicopter.surub;
import org.firstinspires.ftc.teamcode.recunoștință.Detection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "autonomie")
public class doarParcare extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        surub servouri = new surub();
        Detection detection = new Detection();

        detection.VisionInitialization(hardwareMap, telemetry);
        servouri.autoINIT(hardwareMap);

        Pose2d start = new Pose2d(0,0,Math.toRadians(90));
        drive.setPoseEstimate(start);


        TrajectorySequence CAZ1 = drive.trajectorySequenceBuilder(start)
                .strafeLeft(30)
                .forward(15)
                        .build();

        Trajectory CAZ2 = drive.trajectoryBuilder(start)
                .forward(15)
                .build();

        TrajectorySequence CAZ3 = drive.trajectorySequenceBuilder(start)
                .strafeRight(30)
                .forward(15)
                .build();

        waitForStart();

        detection.detectare(telemetry);

        if(opModeIsActive()){
            servouri.cleste.setPosition(0.45);
            sleep(200);
            servouri.turn.setPosition(0.3);
            if (detection.tagOfInterest.id == detection.Caz1) {
                drive.followTrajectorySequence(CAZ1);
            }
            else if (detection.tagOfInterest.id == detection.Caz3) {
                drive.followTrajectorySequence(CAZ3);
            }
            else {
                drive.followTrajectory(CAZ2);
            }
            sleep(500);
        }}

    }
