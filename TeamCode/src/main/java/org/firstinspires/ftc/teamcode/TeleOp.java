package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

import org.robotoasters.ftc.helper.BeaconHandler;


/**
 * Created by team 11735/8487 on 9/27/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group="Linear Opmode")

public class TeleOp extends OpMode{

    BeaconHandler beaconPush;
    DcMotor motorLeft1;
    DcMotor motorLeft2;
    DcMotor motorRight1;
    DcMotor motorRight2;

    DcMotor Lift1;
    DcMotor Lift2;

    DcMotor LClaw;
    DcMotor RClaw;

    static final int LED_CHANNEL = 5;
    private float hsvValues[] = {0F,0F,0F};
    final float values[]  = hsvValues;

    protected float leftY;
    protected float rightY;
    protected float pSquared;

    protected double curLiftPwr;
    protected boolean liftPwrChange = false;

    DeviceInterfaceModule cdim;
    ColorSensor sensorRGB;

    @Override
    public void init() {
        //init motors
        motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight1 = hardwareMap.dcMotor.get("motorRight1");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");

        DcMotor.RunMode mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;

        motorLeft1.setMode(mode);
        motorLeft2.setMode(mode);
        motorRight1.setMode(mode);
        motorRight2.setMode(mode);

        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        sensorRGB = hardwareMap.colorSensor.get("color");

        mode = DcMotor.RunMode.RUN_USING_ENCODER;

        motorLeft1.setMode(mode);
        motorLeft2.setMode(mode);
        motorRight1.setMode(mode);
        motorRight2.setMode(mode);

        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorRight1.setDirection(DcMotor.Direction.FORWARD);
        motorRight2.setDirection(DcMotor.Direction.FORWARD);

        //init Lifts
        Lift1 = hardwareMap.dcMotor.get("Lift1");
        Lift2 = hardwareMap.dcMotor.get("Lift2");

        mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        Lift1.setMode(mode);
        Lift2.setMode(mode);
        //idle();

        mode = DcMotor.RunMode.RUN_USING_ENCODER;
        Lift1.setMode(mode);
        Lift2.setMode(mode);
        Lift1.setDirection(DcMotor.Direction.FORWARD);
        Lift2.setDirection(DcMotor.Direction.REVERSE);
        curLiftPwr = .75;

        //init Claws
        RClaw = hardwareMap.dcMotor.get("RClaw");
        LClaw = hardwareMap.dcMotor.get("LClaw");
        mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        RClaw.setMode(mode);
        LClaw.setMode(mode);

        mode = DcMotor.RunMode.RUN_USING_ENCODER;
        RClaw.setMode(mode);
        LClaw.setMode(mode);
        RClaw.setDirection(DcMotor.Direction.REVERSE);
        LClaw.setDirection(DcMotor.Direction.FORWARD);

        beaconPush = new BeaconHandler(hardwareMap);
    }

    @Override
    public void loop() {
        //Drive Train
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

        // Lift
        if(gamepad2.dpad_up){
            Lift1.setPower(curLiftPwr);
            Lift2.setPower(curLiftPwr);
        }else if(gamepad2.dpad_down){// && (Lift1.getCurrentPosition() > 0)){
            Lift1.setPower(-1*curLiftPwr);
            Lift2.setPower(-1*curLiftPwr);
        }else{
            Lift1.setPower(0);
            Lift2.setPower(0);
        }

        //liftPower
        if(gamepad2.dpad_right && (curLiftPwr < 1) && !liftPwrChange){
            liftPwrChange = true;
            curLiftPwr += .1;
            if(curLiftPwr > 1){
                curLiftPwr = 1;
            }
        }
        if(gamepad2.dpad_left && (curLiftPwr > 0) && !liftPwrChange){
            liftPwrChange = true;
            curLiftPwr -= .1;
            if(curLiftPwr < 0){
                curLiftPwr =0;
            }
        }
        if(!gamepad2.dpad_right && !gamepad2.dpad_left){
            liftPwrChange = false;
        }

        if(gamepad2.right_bumper) {
            RClaw.setPower(.25);
            LClaw.setPower(.25);

        }else if (gamepad2.left_bumper){
            RClaw.setPower(-.25);
            LClaw.setPower(-.25);
        }else{
            RClaw.setPower(0);
            LClaw.setPower(0);

        }
        // Pressing Beacons
        if (gamepad2.b){
            beaconPush.pressBeacon(beaconPush.RED);
        }
        else if (gamepad2.x){
            beaconPush.pressBeacon(beaconPush.BLUE);
        }
        else if (gamepad2.y){
            beaconPush.pressBeacon(3);
        }

        // /Debug
        /*
        telemetry.addData("LeftY",  " at %7d",
                leftY);
        telemetry.addData("RightY",  " at %7d",
                rightY);
        telemetry.update();
        */
        telemetry.addData("motorRight",  " at %7d",
                motorRight1.getCurrentPosition());

/*
        telemetry.addData("Lift1",  " at %7d",
                Lift1.getCurrentPosition());
        telemetry.addData("Lift2",  " at %7d",
                Lift2.getCurrentPosition());
        //telemetry.addData("LiftPwr:", "%7d",
        //        curLiftPwr);
        telemetry.update();
*/
    }
}
