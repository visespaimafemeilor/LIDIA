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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "autonomie")
public class agresivBlue extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        surub servouri = new surub();
        slide slide = new slide();

        servouri.autoINIT(hardwareMap);
        slide.sliderINIT(hardwareMap);

        Pose2d start = new Pose2d(0,0,Math.toRadians(90));

        drive.setPoseEstimate(start);

        TrajectorySequence bagaTare = drive.trajectorySequenceBuilder(start)
                .addDisplacementMarker(0, ()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(200);
                    servouri.turn.setPosition(0.35);
                })
                .forward(20)
                .turn(Math.toRadians(180))
                .back(70)
                .addDisplacementMarker(20, ()-> {
                    slide.autonomousSlider(-1100,1);
                    sleep(100);
                    servouri.autonomousSwing(65,1);
                })
                .build();

        Trajectory parcare = drive.trajectoryBuilder(bagaTare.end())
                .forward(20)
                .build();

        waitForStart();

        drive.followTrajectorySequence(bagaTare);
        sleep(200);
        servouri.cleste.setPosition(0.7);
        sleep(200);
        servouri.autonomousSwing(0,0.5);
        slide.autonomousSlider(0,0.3);
        while (slide.slide.getCurrentPosition()<0){}
        sleep(500);
        drive.followTrajectory(parcare);
        sleep(200);

    }
}