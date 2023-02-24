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

public class renastereaOriginal extends LinearOpMode {
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

        //TODO preload si aliniere cu stiva

        TrajectorySequence preload = drive.trajectorySequenceBuilder(start)
                .lineToConstantHeading(new Vector2d(33,0))
                .addDisplacementMarker(1, ()->{
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
                    servouri.cleste.setPosition(0.65);
                    sleep(200);
                    servouri.autonomousSwing(0,0.5);
                })
                .lineToLinearHeading(new Pose2d(53.2,0,Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(53.2,25))
                .addDisplacementMarker(2, ()->{
                    servouri.turn.setPosition(0.5);
                    slide.autonomousSlider(-230,1);
                })
                .build();

        //TODO first high

        TrajectorySequence first = drive.trajectorySequenceBuilder(stack.end())
                .addDisplacementMarker(0, ()->{
                    servouri.cleste.setPosition(0.45);
                    sleep(100);
                    slide.autonomousSlider(-1115,1);
                    servouri.turn.setPosition(0.4);
                    servouri.autonomousSwing(-85,0.7);
                })
                //.lineToConstantHeading(new Vector2d(60,-12))
                .lineTo(
                        new Vector2d(60, -12),
                        SampleMecanumDrive.getVelocityConstraint(slowerVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(slowerAcceleration)
                )
                .build();

        TrajectorySequence pune = drive.trajectorySequenceBuilder(first.end())
                .addDisplacementMarker(()->{
                    servouri.cleste.setPosition(0.65);
                })
                .waitSeconds(0.2)
                .lineToConstantHeading(new Vector2d(53.2,10))
                .addDisplacementMarker(3, ()->{
                    servouri.turn.setPosition(0.5);
                    servouri.autonomousSwing(0,0.4);
                    slide.autonomousSlider(-140,0.3);
                })
                .build();

        waitForStart();

        if(opModeIsActive()){
            drive.followTrajectorySequence(preload);
            drive.followTrajectorySequence(stack);
            drive.followTrajectorySequence(first);
            drive.followTrajectorySequence(pune);
            slide.autonomousSlider(0,0.5);
            while(slide.slide.getCurrentPosition()<0)slide.autonomousSlider(0,0.5);
            while(slide.slide.getCurrentPosition()<0){}
        }

    }
}
