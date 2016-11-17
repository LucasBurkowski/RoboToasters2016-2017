package org.robotoasters.ftc.helper;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Lucas on 11/14/2016.
 */

public class BeaconHandler {
    private DeviceInterfaceModule cdim;
    private ColorSensor sensorRGB;
    private Servo beaconright;
    private Servo beaconleft;
    private DcMotor motorLeft1;
    private DcMotor motorLeft2;
    private DcMotor motorRight1;
    private DcMotor motorRight2;
    private HardwareMap Map;
    public static int RED = 0;
    public static int BLUE = 1;

    public BeaconHandler(HardwareMap map){
        Map = map;
        initHandler();
    }

    public void initHandler(){
        cdim = Map.deviceInterfaceModule.get("dim");
        beaconleft = Map.servo.get("Srv1");
        beaconright = Map.servo.get("Srv2");
        sensorRGB = Map.colorSensor.get("color");
        motorLeft1 = Map.dcMotor.get("motorLeft1");
        motorLeft2 = Map.dcMotor.get("motorLeft2");
        motorRight1 = Map.dcMotor.get("motorRight1");
        motorRight2 = Map.dcMotor.get("motorRight2");
        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorRight1.setDirection(DcMotor.Direction.FORWARD);
        motorRight2.setDirection(DcMotor.Direction.FORWARD);
        DcMotor.RunMode current = DcMotor.RunMode.RUN_TO_POSITION;
        motorLeft1.setMode(current);
        motorLeft2.setMode(current);
        motorRight1.setMode(current);
        motorRight2.setMode(current);
    }

    public int getColor(){
        if (sensorRGB.red() > sensorRGB.blue()){
            return RED;
        }
        else if (sensorRGB.blue() > sensorRGB.red()){
            return BLUE;
        }
        else{
            return 3;
        }
    }

    public void pressBeacon(int color){
        switch(color){
            case 0: if (getColor() == BLUE){
                beaconleft.setPosition(1);
                pressBeacon();
            }
                else if (getColor() == RED){
                beaconright.setPosition(0);
                pressBeacon();
            }
                else{
                beaconleft.setPosition(.5);
                beaconright.setPosition(.5);
            }
                break;


            case 1: if (getColor() == RED){
                beaconleft.setPosition(1);
                pressBeacon();
            }
            else if (getColor() == BLUE){
                beaconright.setPosition(0);
                pressBeacon();
            }
            else{
                beaconleft.setPosition(.5);
                beaconright.setPosition(.5);
            }
                break;


            default:
                beaconright.setPosition(.5);
                beaconleft.setPosition(.5);
                break;
        }

    }

    private void pressBeacon(){
        boolean atPosition = false;
        int pushDist = inchesToTicks(5);
        motorLeft1.setTargetPosition(pushDist);
        motorLeft2.setTargetPosition(pushDist);
        motorRight1.setTargetPosition(pushDist);
        motorRight2.setTargetPosition(pushDist);
        while (atPosition == false){
            if (motorRight2.getCurrentPosition() >= pushDist){
                atPosition = true;
            }
            else{
                motorRight2.setPower(-0.5);
                motorRight1.setPower(-0.5);
                motorLeft1.setPower(-0.5);
                motorLeft2.setPower(-0.5);
            }
        }
    }

    private int inchesToTicks(double distance){
        return((int) Math.ceil(
                distance*2880/32
        ));
    }
}
