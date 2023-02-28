/*
package org.firstinspires.ftc.teamcode.AUTONOMIE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.android.dx.rop.cst.CstAnnotation;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.elicopter.surub;
import org.firstinspires.ftc.teamcode.recunoștință.Detection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.elicopter.slide;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "autonomie")

public class normalitate extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        slide slide = new slide();
        surub servouri = new surub();
        servouri.autoINIT(hardwareMap);
        slide.sliderINIT(hardwareMap);
//        Detection detection = new Detection();
//        detection.VisionInitialization(hardwareMap, telemetry);

        Pose2d start = new Pose2d(0,0, Math.toRadians(90));
        drive.setPoseEstimate(start);

        double slowerVelocity = 17;
        double slowerAcceleration = 17;


        //TODO: preload si aliniere cu stiva

        TrajectorySequence preload = drive.trajectorySequenceBuilder(start)
                .lineToConstantHeading(new Vector2d(0,34))
                .addDisplacementMarker(2, ()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(200);
                    servouri.turn.setPosition(0.35);
                    slide.autonomousSlider(-790,1);
                    servouri.autonomousSwing(-65,1);
                })
                .waitSeconds(0.1)
                .build();

        TrajectorySequence stack = drive.trajectorySequenceBuilder(preload.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.7);
                    sleep(200);
                    servouri.autonomousSwing(0,1);
                })
                .lineToLinearHeading(new Pose2d(0,49,Math.toRadians(176)))
                .lineToConstantHeading(new Vector2d(-23.5,50))
                .addDisplacementMarker(5, ()->{
                    servouri.turn.setPosition(0.5);
                    slide.autonomousSlider(-175,1);
                })
                .build();

        //TODO: first high

        TrajectorySequence first = drive.trajectorySequenceBuilder(stack.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(200);
                    slide.autonomousSlider(-1160,1);
                    servouri.turn.setPosition(0.4);
                })
//                .lineTo(
//                        new Vector2d(12, 49),
//                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
//                )
                .lineToConstantHeading(new Vector2d(12,49))
                .addDisplacementMarker(2,()->{
                    servouri.autonomousSwing(-65,0.7);
                })
                .build();

        TrajectorySequence pune = drive.trajectorySequenceBuilder(first.end())
                .addDisplacementMarker(()->{
                    servouri.turn.setPosition(0.5);
                    sleep(100);
                    servouri.cleste.setPosition(0.7);
                    sleep(200);
                    servouri.turn.setPosition(0.3);
                    servouri.autonomousSwing(0,0.4);
                })
                .waitSeconds(0.2)
                .lineToConstantHeading(new Vector2d(-23.5,50))
                .addDisplacementMarker(3, ()->{
                    servouri.turn.setPosition(0.5);
                    slide.autonomousSlider(-140,0.3);
                })
                .build();

        //TODO: second high

        TrajectorySequence second = drive.trajectorySequenceBuilder(pune.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(200);
                    slide.autonomousSlider(-1160,1);
                    servouri.turn.setPosition(0.4);
                })
                .lineTo(
                        new Vector2d(12, 46),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .addDisplacementMarker(2,()->{
                    servouri.autonomousSwing(-65,1);
                })
                .build();

        TrajectorySequence puneDoi = drive.trajectorySequenceBuilder(second.end())
                .addDisplacementMarker(()->{
                    servouri.turn.setPosition(0.5);
                    sleep(100);
                    servouri.cleste.setPosition(0.7);
                    sleep(200);
                    servouri.turn.setPosition(0.3);
                    servouri.autonomousSwing(0,0.4);
                })
                .waitSeconds(0.2)
                .lineToConstantHeading(new Vector2d(-20,46))
                .addDisplacementMarker(4, ()->{
                    servouri.turn.setPosition(0.5);
                    slide.autonomousSlider(-130,0.3);
                })
                .build();

        //TODO: third high

        TrajectorySequence third = drive.trajectorySequenceBuilder(puneDoi.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(200);
                    slide.autonomousSlider(-1160,1);
                    servouri.turn.setPosition(0.4);
                    servouri.autonomousSwing(-65,1);
                })
                .lineToConstantHeading(new Vector2d(3,53))
                .addDisplacementMarker(9,()->{
                    servouri.autonomousSwing(-65,1);
                })
                .build();

        TrajectorySequence puneTrei = drive.trajectorySequenceBuilder(third.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.7);
                    sleep(200);
                    servouri.autonomousSwing(0,0.4);
                })
                .waitSeconds(0.2)
                .lineToConstantHeading(new Vector2d(-35.5,55))
                .addDisplacementMarker(4, ()->{
                    servouri.turn.setPosition(0.5);
                    slide.autonomousSlider(-90,0.3);
                })
                .build();

        //TODO: fourth high

        TrajectorySequence fourth = drive.trajectorySequenceBuilder(puneTrei.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(200);
                    slide.autonomousSlider(-1160,1);
                    servouri.turn.setPosition(0.4);
                    servouri.autonomousSwing(-65,1);
                })
                .lineToConstantHeading(new Vector2d(0,52.5))
                .addDisplacementMarker(9,()->{
                    servouri.autonomousSwing(-65,1);
                })
                .build();

        TrajectorySequence punePatru = drive.trajectorySequenceBuilder(fourth.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.7);
                    sleep(200);
                    servouri.autonomousSwing(0,0.4);
                })
                .waitSeconds(0.2)
                .lineToConstantHeading(new Vector2d(-20,53.5))
                .addDisplacementMarker(4, ()->{
                    servouri.turn.setPosition(0.5);
                    slide.autonomousSlider(-50,0.3);
                })
                .build();

//        //TODO: last on low
//
//        TrajectorySequence low = drive.trajectorySequenceBuilder(punePatru.end())
//                .addDisplacementMarker(()->{
//                    servouri.cleste.setPosition(0.45);
//                    sleep(200);
//                    slide.autonomousSlider(-400,1);
//                    servouri.turn.setPosition(0.28);
//                    servouri.rasucire.setPosition(0);
//                })
//                .lineToConstantHeading(new Vector2d(-15, 48))
//                .waitSeconds(0.2)
//                .addTemporalMarker(()->{
//                    servouri.cleste.setPosition(0.7);
//                    sleep(200);
//                    servouri.rasucire.setPosition(0.5);
//                })
//                .build();

//        //TODO: PARCARE
//
//        TrajectorySequence CAZ1 = drive.trajectorySequenceBuilder(low.end())
//                .forward(10)
//                .strafeLeft(10)
//                .build();
//
//        TrajectorySequence CAZ2 = drive.trajectorySequenceBuilder(low.end())
//                .back(10)
//                .strafeLeft(10)
//                .build();
//
//        TrajectorySequence CAZ3 = drive.trajectorySequenceBuilder(low.end())
//                .back(25)
//                .strafeLeft(10)
//                .build();

        //TODO START
        waitForStart();

//      detection.detectare(telemetry);


        if(opModeIsActive()){
            drive.followTrajectorySequence(preload);
            drive.followTrajectorySequence(stack);
            drive.followTrajectorySequence(first);
            drive.followTrajectorySequence(pune);
            drive.followTrajectorySequence(second);
            drive.followTrajectorySequence(puneDoi);
//            drive.followTrajectorySequence(third);
//            drive.followTrajectorySequence(puneTrei);
//            drive.followTrajectorySequence(fourth);
//            drive.followTrajectorySequence(punePatru);
            servouri.autonomousSwing(0,1);
//            if (detection.tagOfInterest.id == detection.Caz1) {
//                drive.followTrajectorySequence(CAZ1);
//            }
//            else if (detection.tagOfInterest.id == detection.Caz3) {
//                drive.followTrajectorySequence(CAZ3);
//            }
//            else {
//                drive.followTrajectorySequence(CAZ2);
//            }
            slide.autonomousSlider(0,0.5);
            while(slide.slide.getCurrentPosition()<0){}
            sleep(500);
        }}

    //TODO FUNCTII:

}

*/