package org.firstinspires.ftc.teamcode.elicopter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static java.lang.Thread.sleep;

public class slide {

    public DcMotor slide;

    int pozitie = 0;

    public  void sliderINIT(HardwareMap hardwareMap){
        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void pozitiune (int pozitie, double putere) {
        slide.setTargetPosition(pozitie);
        slide.setPower(putere);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void autonomousSlider (int position, double power){
        pozitiune(position,power);
//        while (slide.getCurrentPosition()>position){}
    }

    public void corectie(){
        if (slide.getCurrentPosition()>0){
            pozitie = 0;
        }
        if(slide.getCurrentPosition()<-2000){
            pozitie = -2000;
        }
    }

//    public void sliderControllers(boolean cross, boolean circle, boolean triangle, boolean square)
//            throws InterruptedException {
//        if(triangle) {
//            pozitie = -1155;
//        }//high
//        if(circle){
//            pozitie = -820;
//        }//mid
//        if(cross){
//            pozitie = -410;
//        }//low
//        if(square){
//            pozitie = -50;
//        }//base
//    }
}
