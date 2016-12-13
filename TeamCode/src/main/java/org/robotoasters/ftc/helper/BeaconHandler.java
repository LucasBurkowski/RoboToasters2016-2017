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
    private HardwareMap Map;
    private int LED_CHANNEL = 5;
    public static int RED = 0;
    public static int BLUE = 1;

    public BeaconHandler(HardwareMap map){
        Map = map;
        initHandler();
    }

    private void initHandler(){
        cdim = Map.deviceInterfaceModule.get("dim");
        beaconleft = Map.servo.get("Srv1");
        beaconright = Map.servo.get("Srv2");
        sensorRGB = Map.colorSensor.get("color");
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        cdim.setDigitalChannelState(LED_CHANNEL, false);
    }

    private int getColor(){
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
            }
                else if (getColor() == RED){
                beaconright.setPosition(0);
            }
                else{
                beaconleft.setPosition(.5);
                beaconright.setPosition(.5);
            }
                break;

            case 1: if (getColor() == RED){
                beaconleft.setPosition(1);
            }
            else if (getColor() == BLUE){
                beaconright.setPosition(0);
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

    public int printColor() {
        return getColor();
    }
}
