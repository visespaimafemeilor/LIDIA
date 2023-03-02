package org.firstinspires.ftc.teamcode.AUTONOMIE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.elicopter.slide;
import org.firstinspires.ftc.teamcode.elicopter.surub;
import org.firstinspires.ftc.teamcode.recunoștință.Detection;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "autonomie")
public class midStanga extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d start = new Pose2d(0,0,Math.toRadians(270));
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        slide slide = new slide();
        surub servouri = new surub();
        servouri.autoINIT(hardwareMap);
        slide.sliderINIT(hardwareMap);

        Detection detection = new Detection();
        detection.VisionInitialization(hardwareMap, telemetry);
        drive.setPoseEstimate(start);

        double slowerVelocity = 30;
        double slowerAcceleration = 25;

        //TODO preload si alinierea cu stiva

        TrajectorySequence preload = drive.trajectorySequenceBuilder(start)
                .lineTo(
                        new Vector2d(0, 37.5),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
//                .lineToConstantHeading(new Vector2d(0,37.5))
                .addDisplacementMarker(3, ()->{
                    servouri.turn.setPosition(0.35);
                    slide.autonomousSlider(-840,1);
                    servouri.rasucire.setPosition(0.59);
                })
                .build();

        TrajectorySequence stack = drive.trajectorySequenceBuilder(preload.end())
                .addDisplacementMarker(()->{
                    servouri.turn.setPosition(0.5);
                    sleep(200);
                    servouri.cleste.setPosition(0.6);
                })
                .lineToLinearHeading(new Pose2d(-10,54.2,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-25,53))
                .addDisplacementMarker(3, ()->{
                    slide.pozitiune(-210,1);
                    servouri.rasucire.setPosition(0.5);
                })
                .build();

        //TODO first mid

        TrajectorySequence first = drive.trajectorySequenceBuilder(stack.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(200);
                    servouri.turn.setPosition(0.35);
                    slide.pozitiune(-785,1);
                })
//                .lineToConstantHeading(new Vector2d(57.5,-9.5))
                .lineTo(
                        new Vector2d(9, 48.5),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .addDisplacementMarker(2, ()->{
                    servouri.rasucire.setPosition(0.36);
                })
                .build();

        TrajectorySequence puneUnu = drive.trajectorySequenceBuilder(first.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.65);
                    servouri.turn.setPosition(0.5);
                    sleep(200);
                    servouri.turn.setPosition(0.3);
                })
                .addDisplacementMarker(2, ()->{
                    servouri.turn.setPosition(0.5);
                    servouri.rasucire.setPosition(0.5);
                })
                .addDisplacementMarker(4, ()->{
                    slide.pozitiune(-149,0.8);
                })
                .lineToConstantHeading(new Vector2d(-27,52))
                .build();

        //TODO second mid

        TrajectorySequence second = drive.trajectorySequenceBuilder(puneUnu.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(200);
                    servouri.turn.setPosition(0.35);
                    slide.pozitiune(-785,1);
                })
//                .lineToConstantHeading(new Vector2d(57.5,-9.5))
                .lineTo(
                        new Vector2d(8.5, 48.7),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .addDisplacementMarker(2, ()->{
                    servouri.rasucire.setPosition(0.36);
                })
                .build();

        TrajectorySequence puneDoi = drive.trajectorySequenceBuilder(second.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.65);
                    servouri.turn.setPosition(0.5);
                    sleep(200);
                    servouri.turn.setPosition(0.3);
                })
                .addDisplacementMarker(2, ()->{
                    servouri.turn.setPosition(0.5);
                    servouri.rasucire.setPosition(0.5);
                })
                .addDisplacementMarker(10, ()->{
                    slide.pozitiune(-110,0.8);
                })
                .lineToConstantHeading(new Vector2d(-26.5,52))
                .build();

        //TODO third mid

        TrajectorySequence third = drive.trajectorySequenceBuilder(puneDoi.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(200);
                    servouri.turn.setPosition(0.35);
                    slide.pozitiune(-775,1);
                })
//                .lineToConstantHeading(new Vector2d(57.5,-9.5))
                .lineTo(
                        new Vector2d(9, 48.5),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .addDisplacementMarker(2, ()->{
                    servouri.rasucire.setPosition(0.36);
                })
                .build();

        TrajectorySequence puneTrei = drive.trajectorySequenceBuilder(third.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.65);
                    servouri.turn.setPosition(0.5);
                    sleep(200);
                    servouri.turn.setPosition(0.3);
                })
                .addDisplacementMarker(2, ()->{
                    servouri.turn.setPosition(0.5);
                    servouri.rasucire.setPosition(0.5);
                })
                .addDisplacementMarker(10, ()->{
                    slide.pozitiune(-55,0.8);
                })
                .lineToConstantHeading(new Vector2d(-26.5,52))
                .build();

        //TODO fourth mid

        TrajectorySequence fourth = drive.trajectorySequenceBuilder(puneTrei.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(200);
                    servouri.turn.setPosition(0.35);
                    slide.pozitiune(-775,1);
                })
//                .lineToConstantHeading(new Vector2d(57.5,-9.5))
                .lineTo(
                        new Vector2d(8.8, 48.5),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .addDisplacementMarker(2, ()->{
                    servouri.rasucire.setPosition(0.36);
                })
                .build();

        TrajectorySequence punePatru = drive.trajectorySequenceBuilder(fourth.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.65);
                    servouri.turn.setPosition(0.5);
                    sleep(200);
                    servouri.turn.setPosition(0.3);
                })
                .addDisplacementMarker(2, ()->{
                    servouri.turn.setPosition(0.5);
                    servouri.rasucire.setPosition(0.5);
                })
                .addDisplacementMarker(10, ()->{
                    slide.pozitiune(0,0.8);
                })
                .lineToConstantHeading(new Vector2d(-26.5,52))
                .build();

        //TODO fifth mid

        TrajectorySequence fifth = drive.trajectorySequenceBuilder(punePatru.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(200);
                    servouri.turn.setPosition(0.35);
                    slide.pozitiune(-790,1);
                })
//                .lineToConstantHeading(new Vector2d(57.5,-9.5))
                .lineTo(
                        new Vector2d(8, 48),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .addDisplacementMarker(2, ()->{
                    servouri.rasucire.setPosition(0.36);
                })
                .build();

        TrajectorySequence puneCinci = drive.trajectorySequenceBuilder(fifth.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.65);
                    servouri.turn.setPosition(0.5);
                    sleep(200);
                    servouri.turn.setPosition(0.3);
                })
                .addDisplacementMarker(2, ()->{
                    servouri.rasucire.setPosition(0.5);
                })
                .lineToConstantHeading(new Vector2d(8,52))
                .build();

        //TODO CAZUL 1

        Trajectory CAZ1 = drive.trajectoryBuilder(puneCinci.end())
                .lineToConstantHeading(new Vector2d(20,52))
                .build();

        //TODO CAZUL 2

        Trajectory CAZ2 = drive.trajectoryBuilder(puneCinci.end())
                .lineToConstantHeading(new Vector2d(1,52))
                .build();

        //TODO CAZUL 3

        Trajectory CAZ3 = drive.trajectoryBuilder(puneCinci.end())
                .lineToConstantHeading(new Vector2d(-26,51))
                .build();


        waitForStart();

        detection.detectare(telemetry);

        if(opModeIsActive()){
            drive.followTrajectorySequence(preload);
            drive.followTrajectorySequence(stack);
            drive.followTrajectorySequence(first);
            drive.followTrajectorySequence(puneUnu);
            drive.followTrajectorySequence(second);
            drive.followTrajectorySequence(puneDoi);
            drive.followTrajectorySequence(third);
            drive.followTrajectorySequence(puneTrei);
            drive.followTrajectorySequence(fourth);
            drive.followTrajectorySequence(punePatru);
            drive.followTrajectorySequence(fifth);
            drive.followTrajectorySequence(puneCinci);

            if (detection.CAZ == 1) {
                drive.followTrajectory(CAZ1);
            } else if (detection.CAZ == 2) {
                drive.followTrajectory(CAZ2);
            } else {
                drive.followTrajectory(CAZ3);
            }

            slide.autonomousSlider(0,0.5);
            while(slide.slide.getCurrentPosition()<0){}
        }
    }
}
