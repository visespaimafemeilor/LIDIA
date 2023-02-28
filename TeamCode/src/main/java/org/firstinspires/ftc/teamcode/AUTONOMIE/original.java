/*
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

        double slowerVelocity = 20;
        double slowerAcceleration = 20;

        //TODO preload  si aliniere cu stiva

        TrajectorySequence preload = drive.trajectorySequenceBuilder(start)
                .addDisplacementMarker( ()->{
                    servouri.cleste.setPosition(0.4);
                })
                .lineTo(
                        new Vector2d(37.5, 0),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .addDisplacementMarker(3, ()->{
                    servouri.turn.setPosition(0.4);
                    slide.autonomousSlider(-850,1);
                    servouri.rasucire.setPosition(0.59);
                })
                .build();

        TrajectorySequence stack = drive.trajectorySequenceBuilder(preload.end())
                .addDisplacementMarker(()->{
                    servouri.turn.setPosition(0.5);
                    sleep(200);
                    servouri.cleste.setPosition(0.6);
                })
                .lineToLinearHeading(new Pose2d(53,5,Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(53,25.5))
                .addDisplacementMarker(3, ()->{
                    slide.autonomousSlider(-230,1);
                    servouri.rasucire.setPosition(0.5);
                })
                .build();

        //TODO first high

        TrajectorySequence first = drive.trajectorySequenceBuilder(stack.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(200);
                    servouri.turn.setPosition(0.4);
                    slide.autonomousSlider(-1175,1);
                })
//                .lineToConstantHeading(new Vector2d(57.5,-9.5))
                .lineTo(
                        new Vector2d(54.5, -6.5),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .addDisplacementMarker(2, ()->{

                    servouri.rasucire.setPosition(0.6);
                })
                .build();

        TrajectorySequence puneUnu = drive.trajectorySequenceBuilder(first.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.65);
                })
                .lineToConstantHeading(new Vector2d(51,27.5))
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{servouri.autonomousSwing(0,0.8);})
                .addDisplacementMarker(2, ()->{
                    slide.autonomousSlider(-180,0.4);
                    servouri.turn.setPosition(0.5);
                })
                .build();

        //TODO second high

        TrajectorySequence second = drive.trajectorySequenceBuilder(puneUnu.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(300);
                    servouri.turn.setPosition(0.4);
                    slide.autonomousSlider(-1175,1);
                })
//                .lineToConstantHeading(new Vector2d(57.5,-9.5))
                .lineTo(
                        new Vector2d(53, -5.5),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .addDisplacementMarker(2, ()->{

                    servouri.autonomousSwing(-85,0.7);
                })
                .build();

        TrajectorySequence puneDoi = drive.trajectorySequenceBuilder(second.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.65);
                })
                .lineToConstantHeading(new Vector2d(51,29))
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{servouri.autonomousSwing(0,0.8);})
                .addDisplacementMarker(3, ()->{
                    servouri.turn.setPosition(0.5);
                    slide.autonomousSlider(-140,0.4);
                })
                .build();

        //TODO third high

        TrajectorySequence third = drive.trajectorySequenceBuilder(puneDoi.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(300);
                    slide.autonomousSlider(-1175,1);
                })
//                .lineToConstantHeading(new Vector2d(57.5,-9.5))
                .lineTo(
                        new Vector2d(51.3, -4.5),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .addDisplacementMarker(2, ()->{
                    servouri.turn.setPosition(0.4);
                    servouri.autonomousSwing(-85,0.7);
                })
                .build();

        TrajectorySequence puneTrei = drive.trajectorySequenceBuilder(third.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.65);
                })
                .lineToConstantHeading(new Vector2d(49,30.5))
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{servouri.autonomousSwing(0,0.8);})
                .addDisplacementMarker(3, ()->{
                    servouri.turn.setPosition(0.5);
                    slide.autonomousSlider(-90,0.3);
                })
                .build();

        //TODO first low

        TrajectorySequence fourth = drive.trajectorySequenceBuilder(puneTrei.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(300);
                    slide.autonomousSlider(-440,1);
                    servouri.turn.setPosition(0.4);
                    servouri.autonomousSwing(85,0.7);
                })
//                .lineToConstantHeading(new Vector2d(57.5,-9.5))
                .lineTo(
                        new Vector2d(45, 20),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .build();

        TrajectorySequence punePatru = drive.trajectorySequenceBuilder(fourth.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.65);
                })
                .waitSeconds(0.2)
                .lineToConstantHeading(new Vector2d(46,20))
                .addDisplacementMarker(3, ()->{
                    //servouri.turn.setPosition(0.5);
                    servouri.autonomousSwing(0,0.8);
                    slide.autonomousSlider(0,0.4);
                })
                .build();


        waitForStart();

        if(opModeIsActive()){
            //TODO MID
            drive.followTrajectorySequence(preload);
            drive.followTrajectorySequence(stack);

            //TODO HIGH
            drive.followTrajectorySequence(first);
            drive.followTrajectorySequence(puneUnu);
            drive.followTrajectorySequence(second);
            drive.followTrajectorySequence(puneDoi);
            drive.followTrajectorySequence(third);
            drive.followTrajectorySequence(puneTrei);

            //TODO LOW
            drive.followTrajectorySequence(fourth);
            drive.followTrajectorySequence(punePatru);

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
*/
