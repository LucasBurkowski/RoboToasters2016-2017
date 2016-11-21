package org.robotoasters.ftc.helper;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by team 11735/8487 on 11/16/2016.
 */

public class NavigationHandler {

    private HardwareMap Map;
    DcMotor motorLeft1;
    DcMotor motorLeft2;
    DcMotor motorRight1;
    DcMotor motorRight2;

    public NavigationHandler(HardwareMap Map) {
        this.Map = Map;
        motorLeft1 = Map.dcMotor.get("motorLeft1");
        motorLeft2 = Map.dcMotor.get("motorLeft2");
        motorRight1 = Map.dcMotor.get("motorRight1");
        motorRight2 = Map.dcMotor.get("motorRight2");
        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorRight1.setDirection(DcMotor.Direction.FORWARD);
        motorRight2.setDirection(DcMotor.Direction.FORWARD);
    }







}
