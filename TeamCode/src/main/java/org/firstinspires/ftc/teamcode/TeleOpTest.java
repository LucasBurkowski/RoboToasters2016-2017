package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by jacost63 on 9/27/2016.
 */
@TeleOp(name="TeleOp Test", group="Linear Opmode")

public class TeleOpTest extends OpMode{

    DcMotor motorLeft1;
    DcMotor motorLeft2;
    DcMotor motorRight1;
    DcMotor motorRight2;
    protected float leftY;
    protected float rightY;
    protected float pSquared;


    @Override
    public void init() {
        motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight1 = hardwareMap.dcMotor.get("motorRight1");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");
        DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;
        motorLeft1.setMode(mode);
        motorLeft2.setMode(mode);
        motorRight1.setMode(mode);
        motorRight2.setMode(mode);

        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorRight1.setDirection(DcMotor.Direction.FORWARD);
        motorRight2.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        leftY = gamepad1.left_stick_y;
        rightY = gamepad1.right_stick_y;
        if(leftY >0){
            pSquared = (leftY*leftY);
        }else
        {
            pSquared = -1*(leftY*leftY);
        }

        motorLeft1.setPower(pSquared);
        motorLeft2.setPower(pSquared);
        if(rightY>0){
            pSquared = (rightY*rightY);
        }else{
            pSquared = -1*(rightY*rightY);
        }

        motorRight1.setPower(pSquared);
        motorRight2.setPower(pSquared);
    }
}
