package org.firstinspires.ftc.teamcode.elicopter;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class surub {

    public Servo cleste;
    public Servo turn;
    public Servo rasucire;

//    public DcMotor rasucire;

    public RevColorSensorV3 sensor;

    int curl = 0;


//    public void servoINIT(HardwareMap hardwareMap){
//        cleste = hardwareMap.get(Servo.class, "cleste");
//        rasucire = hardwareMap.get(Servo.class, "rasucire");
//        turn = hardwareMap.get(Servo.class, "turn");
//
//        cleste.setPosition(0.7);
//        turn.setPosition(0.3);
//        rasucire.setPosition(0.5);
//    }

//    public void autoINIT(HardwareMap hardwareMap) throws InterruptedException{
//        cleste = hardwareMap.get(Servo.class, "cleste");
//        rasucire = hardwareMap.get(Servo.class, "rasucire");
//        turn = hardwareMap.get(Servo.class, "turn");
//
//        rasucire.setPosition(0.5);
//        cleste.setPosition(0.8);
//        sleep(500);
//        turn.setPosition(0.5);
//    }

    public void servoINIT(HardwareMap hardwareMap){
        sensor = hardwareMap.get(RevColorSensorV3.class, "senzor");

        cleste = hardwareMap.get(Servo.class, "cleste");
        turn = hardwareMap.get(Servo.class, "turn");
        rasucire = hardwareMap.get(Servo.class,"rasucire");

        cleste.setPosition(0.65);
        turn.setPosition(0.4);
        rasucire.setPosition(0.5);

//        rasucire = hardwareMap.get(DcMotor.class, "rasucire");
//        rasucire.setDirection(DcMotorSimple.Direction.FORWARD);
//        rasucire.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rasucire.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rasucire.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void autoINIT(HardwareMap hardwareMap) throws InterruptedException{
//        sensor = hardwareMap.get(RevColorSensorV3.class, "senzor");

        cleste = hardwareMap.get(Servo.class, "cleste");
        turn = hardwareMap.get(Servo.class, "turn");
        rasucire = hardwareMap.get(Servo.class,"rasucire");

        cleste.setPosition(0.45);
        sleep(200);
        turn.setPosition(0.2);
        rasucire.setPosition(0.5);

//        rasucire = hardwareMap.get(DcMotor.class, "rasucire");
//        rasucire.setDirection(DcMotorSimple.Direction.FORWARD);
//        rasucire.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rasucire.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rasucire.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void apucare(boolean x, boolean y){
        if(x){
            cleste.setPosition(0.45);
        }
        if(y){
            cleste.setPosition(0.65);
        }
    }

    public void ridicare (boolean x, boolean y){
        //sus
        if(x){
            turn.setPosition(0.35);
        }

        //jos
        if(y){
            turn.setPosition(0.5);
        }
    }

//    public void swing (int pozitie, double putere) {
//        rasucire.setTargetPosition(pozitie);
//        rasucire.setPower(putere);
//        rasucire.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    public void autonomousSwing (int position, double power){
//        swing(position,power);
//        while (rasucire.getCurrentPosition()>position){}
//    }
//
//    public void intoarcere (boolean x, boolean y, boolean z){
//        if(x){
//            curl=-85;
//        }
//        if(y){
//            curl=0;
//        }
//        if(z){
//            curl=85;
//        }
//    }

    public boolean acolosa(){
        if(sensor.getDistance(DistanceUnit.CM) <=5){
            return true;
        }
        else
            return false;
    }

    public void intoarcere (boolean x, boolean y, boolean z){
        //stanga robotului
        if(x){
            rasucire.setPosition(0.4);
        }
        //centru
        if(y){
            rasucire.setPosition(0.5);
        }
        //dreapta robotului
        if(z){
            rasucire.setPosition(0.6);
        }
    }

}