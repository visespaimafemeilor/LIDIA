package org.firstinspires.ftc.teamcode.elicopter;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class surub{

    public Servo cleste;
    public Servo turn;
    public Servo rasucire;

    public RevColorSensorV3 sensor;

    int curl = 0;

    public void servoINIT(HardwareMap hardwareMap){
        sensor = hardwareMap.get(RevColorSensorV3.class, "senzor");

        cleste = hardwareMap.get(Servo.class, "cleste");
        turn = hardwareMap.get(Servo.class, "turn");
        rasucire = hardwareMap.get(Servo.class,"rasucire");

        cleste.setPosition(0.65);
        turn.setPosition(0.35);
        rasucire.setPosition(0.5);
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