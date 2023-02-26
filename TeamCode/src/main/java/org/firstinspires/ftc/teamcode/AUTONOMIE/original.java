package org.firstinspires.ftc.teamcode.AUTONOMIE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.elicopter.slide;
import org.firstinspires.ftc.teamcode.elicopter.surub;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "autonomie")

public class original extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d start = new Pose2d(0,0,0);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        slide slide = new slide();
        surub servouri = new surub();
        servouri.autoINIT(hardwareMap);
        slide.sliderINIT(hardwareMap);

        drive.setPoseEstimate(start);

        double slowerVelocity = 25;
        double slowerAcceleration = 20;

        //TODO preload  si aliniere cu stiva

        TrajectorySequence preload = drive.trajectorySequenceBuilder(start)
                .lineTo(
                        new Vector2d(37, 1),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .addDisplacementMarker(1, ()->{
                    servouri.cleste.setPosition(0.4);
                    sleep(300);
                    servouri.turn.setPosition(0.4);
                    slide.autonomousSlider(-835,1);
                    servouri.autonomousSwing(-65,1);
                })
                .build();

        TrajectorySequence stack = drive.trajectorySequenceBuilder(preload.end())
                .addDisplacementMarker(()->{
                    servouri.turn.setPosition(0.5);
                    sleep(200);
                    servouri.cleste.setPosition(0.6);
                    sleep(200);
                    //servouri.turn.setPosition(0.4);
                    servouri.autonomousSwing(0,0.3);
                })
                .lineToLinearHeading(new Pose2d(53,5,Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(53,26))
                .addDisplacementMarker(2, ()->{
                    //servouri.turn.setPosition(0.5);
                    slide.autonomousSlider(-225,1);
                })
                .build();

        //TODO first high

        TrajectorySequence first = drive.trajectorySequenceBuilder(stack.end())
                .addDisplacementMarker(0, ()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(300);
                    servouri.turn.setPosition(0.4);
                    slide.autonomousSlider(-1150,1);
                    servouri.autonomousSwing(-95,0.7);
                })
//                .lineToConstantHeading(new Vector2d(57.5,-9.5))
                .lineTo(
                        new Vector2d(56.5, -7),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )

                .build();

        TrajectorySequence puneUnu = drive.trajectorySequenceBuilder(first.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.6);
                })
                .waitSeconds(0.2)
                .lineToConstantHeading(new Vector2d(53,26.5))
                .addDisplacementMarker(3, ()->{
                    servouri.turn.setPosition(0.5);
                    servouri.autonomousSwing(0,0.7);
                    slide.autonomousSlider(-200,0.3);
                })
                .build();

        //TODO second high

        TrajectorySequence second = drive.trajectorySequenceBuilder(puneUnu.end())
                .addDisplacementMarker(0, ()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(300);
                    servouri.turn.setPosition(0.4);
                    slide.autonomousSlider(-1150,1);
                    servouri.autonomousSwing(-95,0.7);
                })
//                .lineToConstantHeading(new Vector2d(57.5,-9.5))
                .lineTo(
                        new Vector2d(57, -6),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )

                .build();

        TrajectorySequence puneDoi = drive.trajectorySequenceBuilder(second.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.6);
                })
                .waitSeconds(0.2)
                .lineToConstantHeading(new Vector2d(53,10))
                .addDisplacementMarker(3, ()->{
                    servouri.turn.setPosition(0.5);
                    servouri.autonomousSwing(0,1);
                    slide.autonomousSlider(-200,0.3);
                })
                .build();


        waitForStart();

        if(opModeIsActive()){
            drive.followTrajectorySequence(preload);
            drive.followTrajectorySequence(stack);
            drive.followTrajectorySequence(first);
            drive.followTrajectorySequence(puneUnu);
            drive.followTrajectorySequence(second);
            drive.followTrajectorySequence(puneDoi);

            slide.autonomousSlider(0,0.5);
            while(slide.slide.getCurrentPosition()<0){}
        }

    }

    //TODO> funcțiuni

//    public void punere(SampleMecanumDrive drive, int position, surub servouri, Pose2d pos, slide slide){
//        TrajectorySequence puneUnu = drive.trajectorySequenceBuilder(pos)
//                .addDisplacementMarker(()->{
//                    servouri.cleste.setPosition(0.6);
//                })
//                .waitSeconds(0.2)
//                .lineToConstantHeading(new Vector2d(53,26))
//                .addDisplacementMarker(3, ()->{
//                    servouri.turn.setPosition(0.5);
//                    servouri.autonomousSwing(0,0.4);
//                    slide.autonomousSlider(position,0.3);
//                })
//                .build();
//        drive.followTrajectorySequence(puneUnu);
//    }
}
