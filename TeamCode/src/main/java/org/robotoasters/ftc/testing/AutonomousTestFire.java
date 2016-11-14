package org.robotoasters.ftc.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Created by jacost63 on 9/27/2016.
 */
@Autonomous(name="AutonomousTestFire", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class AutonomousTestFire extends OpMode {
    private static final int ENCODER_PULSES_PER_REV = 280;

    private static final double ZERO_POWER = 0;
    private static final double MAX_FORWARD = 0.25;
    private static final double MAX_REVERSE = -0.25;
    private DcMotor motorLeft1 = null;
    private DcMotor motorLeft2 = null;
    private DcMotor motorRight1 = null;
    private DcMotor motorRight2 = null;

    @Override
    public void init(){
        this.motorLeft1 = this.hardwareMap.dcMotor.get("motorLeft1");
        this.motorLeft2 = this.hardwareMap.dcMotor.get("motorLeft2");
        this.motorRight1 = this.hardwareMap.dcMotor.get("motorRight1");
        this.motorRight2 = this.hardwareMap.dcMotor.get("motorRight2");
        DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;//RUN_USING_ENCODER;
        this.motorLeft1.setMode(mode);
        this.motorLeft2.setMode(mode);
        this.motorRight1.setMode(mode);
        this.motorRight2.setMode(mode);

        this.motorLeft1.setDirection(DcMotor.Direction.FORWARD);
        this.motorLeft2.setDirection(DcMotor.Direction.FORWARD);
        this.motorRight1.setDirection(DcMotor.Direction.REVERSE);
        this.motorRight2.setDirection(DcMotor.Direction.REVERSE);

        AllStop();
    }

    @Override
    public void loop(){
        for(int i=0;i<10;i++) {
            AllStop();
            MaxForward();
            try {
                Thread.sleep(3000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            AllStop();
            HardRight();//HardRight();
            try {
                Thread.sleep(1250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
    private void Sleep(long sTime){
        long msSleep = System.currentTimeMillis()+sTime;
        while(msSleep < System.currentTimeMillis()){}
    }
    private void MaxForward(){
        this.motorRight1.setPower(MAX_FORWARD);
        this.motorRight2.setPower(MAX_FORWARD);
        this.motorLeft1.setPower(MAX_FORWARD);
        this.motorLeft2.setPower(MAX_FORWARD);
    }
    private void MaxReverse(){
        this.motorRight1.setPower(MAX_REVERSE);
        this.motorRight2.setPower(MAX_REVERSE);
        this.motorLeft1.setPower(MAX_REVERSE);
        this.motorLeft2.setPower(MAX_REVERSE);
    }
    private void HardRight(){
        this.motorRight1.setPower(MAX_REVERSE);
        this.motorRight2.setPower(MAX_REVERSE);
        this.motorLeft1.setPower(MAX_FORWARD);
        this.motorLeft2.setPower(MAX_FORWARD);
    }
    private void HardLeft(){
        this.motorRight1.setPower(MAX_FORWARD);
        this.motorRight2.setPower(MAX_FORWARD);
        this.motorLeft1.setPower(MAX_REVERSE);
        this.motorLeft2.setPower(MAX_REVERSE);
    }
    private void AllStop(){
        this.motorLeft1.setPower(ZERO_POWER);
        this.motorLeft2.setPower(ZERO_POWER);
        this.motorRight1.setPower(ZERO_POWER);
        this.motorRight2.setPower(ZERO_POWER);
    }
}
