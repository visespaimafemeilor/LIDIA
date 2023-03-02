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
public class highDreapta extends LinearOpMode {
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
        servouri.autoINIT(hardwareMap);

        drive.setPoseEstimate(start);

        double slowerVelocity = 30;
        double slowerAcceleration = 25;

        //TODO preload si alinierea cu stiva

        TrajectorySequence preload = drive.trajectorySequenceBuilder(start)
                .lineTo(
                        new Vector2d(0, -37.5),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
//                .lineToConstantHeading(new Vector2d(0,37.5))
                .addDisplacementMarker(3, ()->{
                    servouri.turn.setPosition(0.35);
                    slide.autonomousSlider(-840,1);
                    servouri.rasucire.setPosition(0.41);
                })
                .build();

        TrajectorySequence stack = drive.trajectorySequenceBuilder(preload.end())
                .addDisplacementMarker(()->{
                    servouri.turn.setPosition(0.5);
                    sleep(200);
                    servouri.cleste.setPosition(0.6);
                })
                .lineToLinearHeading(new Pose2d(-10,-54.2,Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-25,-54.2))
                .addDisplacementMarker(3, ()->{
                    slide.pozitiune(-210,1);
                    servouri.rasucire.setPosition(0.5);
                })
                .build();

        //TODO first high

        TrajectorySequence first = drive.trajectorySequenceBuilder(stack.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(200);
                    servouri.turn.setPosition(0.35);
                    slide.pozitiune(-1140,1);
                })
//                .lineToConstantHeading(new Vector2d(57.5,-9.5))
                .lineTo(
                        new Vector2d(8, -57),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .addDisplacementMarker(2, ()->{
                    servouri.rasucire.setPosition(0.35);
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
                    slide.pozitiune(-149,1);
                })
                .lineToConstantHeading(new Vector2d(-27,-52))
                .build();

        //TODO second high

        TrajectorySequence second = drive.trajectorySequenceBuilder(puneUnu.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(200);
                    servouri.turn.setPosition(0.35);
                    slide.pozitiune(-1140,1);
                })
//                .lineToConstantHeading(new Vector2d(57.5,-9.5))
                .lineTo(
                        new Vector2d(8, -57),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .addDisplacementMarker(2, ()->{
                    servouri.rasucire.setPosition(0.35);
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
                .addDisplacementMarker(4, ()->{
                    slide.pozitiune(-110,1);
                })
                .lineToConstantHeading(new Vector2d(-27,-52.5))
                .build();

        //TODO third high

        TrajectorySequence third = drive.trajectorySequenceBuilder(puneDoi.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(200);
                    servouri.turn.setPosition(0.35);
                    slide.pozitiune(-1140,1);
                })
//                .lineToConstantHeading(new Vector2d(57.5,-9.5))
                .lineTo(
                        new Vector2d(8, -56.5),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .addDisplacementMarker(2, ()->{
                    servouri.rasucire.setPosition(0.35);
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
                .addDisplacementMarker(4, ()->{
                    slide.pozitiune(-55,1);
                })
                .lineToConstantHeading(new Vector2d(-27,-52.5))
                .build();

        //TODO fourth high

        TrajectorySequence fourth = drive.trajectorySequenceBuilder(puneTrei.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(200);
                    servouri.turn.setPosition(0.35);
                    slide.pozitiune(-1140,1);
                })
//                .lineToConstantHeading(new Vector2d(57.5,-9.5))
                .lineTo(
                        new Vector2d(8, -56.5),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .addDisplacementMarker(2, ()->{
                    servouri.rasucire.setPosition(0.35);
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
                .addDisplacementMarker(4, ()->{
                    slide.pozitiune(0,1);
                })
                .lineToConstantHeading(new Vector2d(-27,-52))
                .build();

        //TODO fifth high

        TrajectorySequence fifth = drive.trajectorySequenceBuilder(punePatru.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(200);
                    servouri.turn.setPosition(0.35);
                    slide.pozitiune(-1140,1);
                })
//                .lineToConstantHeading(new Vector2d(57.5,-9.5))
                .lineTo(
                        new Vector2d(8, -56.5),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .addDisplacementMarker(2, ()->{
                    servouri.rasucire.setPosition(0.35);
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
                .addDisplacementMarker(4, ()->{
                    slide.pozitiune(0,0.8);
                })
                .lineToConstantHeading(new Vector2d(8,-52))
                .build();

        //TODO CAZUL 1

        Trajectory CAZ1 = drive.trajectoryBuilder(puneCinci.end())
                .lineToConstantHeading(new Vector2d(-26,-51))
                .build();

        //TODO CAZUL 2

        Trajectory CAZ2 = drive.trajectoryBuilder(puneCinci.end())
                .lineToConstantHeading(new Vector2d(1,-51))
                .build();

        //TODO CAZUL 3

        Trajectory CAZ3 = drive.trajectoryBuilder(puneCinci.end())
                .lineToConstantHeading(new Vector2d(19,-51))
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
