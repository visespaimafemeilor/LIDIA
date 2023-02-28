package org.firstinspires.ftc.teamcode.elicopter;

import static java.lang.Thread.sleep;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "teleop")
public class telop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        HardwareMap map = hardwareMap;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        surub surub = new surub();
        slide slide = new slide();

        slide.sliderINIT(hardwareMap);
        surub.servoINIT(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            drive.setWeightedDrivePower(new Pose2d(

                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x
            ));

            slide.pozitiune(slide.pozitie,1);
            slide.pozitie+=(gamepad2.left_trigger-gamepad2.right_trigger)*5;
            slide.sliderControllers(gamepad2.cross,gamepad2.circle,gamepad2.triangle,gamepad2.square);
            slide.corectie();



//            if(surub.cleste.getPosition()<0.5 && slide.slide.getCurrentPosition()>-15){
//                this.sleep(100);
//                surub.turn.setPosition(0.4);
//            }

            surub.apucare(gamepad1.right_bumper, gamepad1.left_bumper);
            surub.ridicare(gamepad2.left_bumper, gamepad2.right_bumper);
            surub.intoarcere(gamepad2.dpad_right,gamepad2.dpad_up, gamepad2.dpad_left);



            if(slide.slide.getCurrentPosition()>-200){
                surub.rasucire.setPosition(0.5);
            }

//            if(slide.slide.getCurrentPosition()>-300){
//                surub.curl = 0;
//            }
//
//            surub.swing(surub.curl,1);
//            surub.curl+=(gamepad1.left_trigger-gamepad1.right_trigger);


//            telemetry.addData("leftFront ", drive.leftFront.getCurrentPosition());
//            telemetry.addData("leftRear ", drive.leftRear.getCurrentPosition());
//            telemetry.addData("rightRear ", drive.rightRear.getCurrentPosition());
//            telemetry.addData("rightFront ", drive.rightFront.getCurrentPosition());
            telemetry.addData("distanta = ", surub.acolosa());
            telemetry.addData("culisanta ", slide.slide.getCurrentPosition());
            telemetry.addData("cleste ", surub.cleste.getPosition());
            telemetry.addData("rasucire ", surub.rasucire.getPosition());
            telemetry.addData("ridicare ", surub.turn.getPosition());
            telemetry.update();
        }
    }
}
