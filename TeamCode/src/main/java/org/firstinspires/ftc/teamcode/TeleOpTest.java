package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by jacost63 on 9/27/2016.
 */
@TeleOp(name="TeleOp Test", group="Linear Opmode")

public class TeleOpTest extends OpMode{

    DcMotor motorLeft1 = null;
    DcMotor motorLeft2 = null;
    DcMotor motorRight1 = null;
    DcMotor motorRight2 = null;
    protected float leftY;
    protected float rightY;


    @Override
    public void init() {
        this.motorLeft1 = this.hardwareMap.dcMotor.get("motorLeft1");
        this.motorLeft2 = this.hardwareMap.dcMotor.get("motorLeft2");
        this.motorRight1 = this.hardwareMap.dcMotor.get("motorRight1");
        this.motorRight2 = this.hardwareMap.dcMotor.get("motorRight2");
        DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;
        this.motorLeft1.setMode(mode);
        this.motorLeft2.setMode(mode);
        this.motorRight1.setMode(mode);
        this.motorRight2.setMode(mode);

        this.motorLeft1.setDirection(DcMotor.Direction.FORWARD);
        this.motorLeft2.setDirection(DcMotor.Direction.FORWARD);
        this.motorRight1.setDirection(DcMotor.Direction.REVERSE);
        this.motorRight2.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        leftY = -gamepad1.left_stick_y;
        rightY = -gamepad1.right_stick_y;

        motorLeft1.setPower(leftY);
        motorLeft2.setPower(leftY);
        motorRight1.setPower(rightY);
        motorRight2.setPower(rightY);
    }
}
