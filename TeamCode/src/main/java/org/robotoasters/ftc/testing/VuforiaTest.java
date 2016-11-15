package org.robotoasters.ftc.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;


/**
 * Created by jacost63 on 9/28/2016.
 * Built for Red Side
 */
@TeleOp(name="VuforiaTestRed", group="Linear Opmode")

public class VuforiaTest extends LinearOpMode {

    private static final int ENCODER_PULSES_PER_REV = 2880;
    private static final double PI = 3.1415;
    private static final double angleError = 5.0;

    DcMotor motorLeft1;
    DcMotor motorLeft2;
    DcMotor motorRight1;
    DcMotor motorRight2;

    protected float leftY;
    protected float rightY;
    protected float pSquared;

    boolean[] targetAcquired;
    boolean[] movingToPosition;





    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "ASWfaHf/////AAAAGZz1J5GuRkProXI9TAVR3P0qosjs6vYAwcHknPmJtBzpsaAvgM6kpgxVHcCMMWVC+61+gDUH9YcPEahwRAIZJPtuiGGxbAQToOO3xOab5i4mIntNLCguABtB7Dc/a4KjbbXDUT/I9dKqCr8flb1DtgfIUheG8GztponPLpeaFKa/Pk03UEixEnTLOcxdz6Y8gfdN4v01FMR7qtDfbsBpi2dz8X+9+SvzfTBkHX5EvmshVgo7fBTVmmB9z7AKtPO1v4lJTxNE/u4zpRIF79FR04/XjqpkMIU4HGg0AY8vmTAnq9qIjhS7FymbKAmbu4bfluZHXsmKEg6Gd9hNXm7spP+hnkNhe9eGdnDR3fTz00C0";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vf = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);

        VuforiaTrackables beacons = vf.loadTrackablesFromAsset("FTC_2016-17");
        //from red side going clockwise
        //http://www.firstinspires.org/sites/default/files/uploads/resource_library/ftc/2016-2017-season/field-setup-guide.pdf
        //pg 12

        beacons.get(0).setName("Gears");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Wheels");

        targetAcquired = new boolean[4];
        movingToPosition = new boolean[4];
        for(int i = 0;i<4;i++){
            targetAcquired[i] = FALSE;
            movingToPosition[i] = FALSE;
        }



        //motor init
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
        idle();

        mode = DcMotor.RunMode.RUN_USING_ENCODER;

        motorLeft1.setMode(mode);
        motorLeft2.setMode(mode);
        motorRight1.setMode(mode);
        motorRight2.setMode(mode);

        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorRight1.setDirection(DcMotor.Direction.FORWARD);
        motorRight2.setDirection(DcMotor.Direction.FORWARD);



        waitForStart();
        
        beacons.activate();
        //0
        //1  is height
        //2   is negative distance
        //bottom is degrees going negative

        while(opModeIsActive()){
            //handle moving
            if(movingToPosition[0] || movingToPosition[1]){

            }
            else{
                //For testing, we let Drive Train happen
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

            //figuring out how to move

            for(VuforiaTrackable beac : beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
                if(pose!=null) {
                    VectorF translation = pose.getTranslation();
                    telemetry.addData(beac.getName() + "Translation", translation);
                    double degreesToTurn = 180-Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn); //for vertial phone. use .get(0), get(2) for horizonal

                    //check if we already have a target
                    if(targetAcquired[0] == TRUE && movingToPosition[0] == FALSE){
                        double turnAngle = getTurnAngle(degreesToTurn);
                        if(turnAngle != 0){
                            movingToPosition[0] = TRUE;
                            setTurn(turnAngle);
                        }else{
                            //turnAngle = 0; should be ready to drive forward.
                        }

                    }
                    else if(targetAcquired[1] == TRUE && movingToPosition[1] ==FALSE){

                    }
                    else{
                        //if we found a new valid beacon
                        if(beac.getName().equals("Gears")) {
                            targetAcquired[0] = TRUE;
                        }
                        else if(beac.getName().equals("Tools")){
                            targetAcquired[1] = TRUE;
                        }
                    }

                }
                else{
                    //no targets in sight, clear all targets
                    for(int i = 0;i<4;i++){
                        targetAcquired[i] = FALSE;
                    }
                }
            }
            telemetry.update();
        }
    }

    double getTurnAngle(double dtt){
        double turnAngle = 0;
        if(dtt<180){
            turnAngle = dtt;
            if(turnAngle < angleError){
                turnAngle = 0;
            }
        }else{
            turnAngle = dtt-360;
            if(turnAngle > -1*angleError){
                turnAngle = 0;
            }
        }
        return turnAngle;
    }
    void setTurn(double t){
        motorLeft1.setPower(0);
        motorLeft2.setPower(0);
        motorRight1.setPower(0);
        motorRight2.setPower(0);

        motorLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;
        motorLeft1.setMode(mode);
        motorLeft2.setMode(mode);
        motorRight1.setMode(mode);
        motorRight2.setMode(mode);

        int distToTravel = inchesToTicks(t);

        motorLeft1.setTargetPosition(distToTravel);
        motorLeft2.setTargetPosition(distToTravel);
        motorRight1.setTargetPosition(-1*distToTravel);
        motorRight2.setTargetPosition(-1*distToTravel);

        mode = DcMotor.RunMode.RUN_TO_POSITION;
        motorLeft1.setMode(mode);
        motorLeft2.setMode(mode);
        motorRight1.setMode(mode);
        motorRight2.setMode(mode);

        motorLeft1.setPower(1);
        motorLeft2.setPower(1);
        motorRight1.setPower(1);
        motorRight2.setPower(1);
    }


    int inchesToTicks(double distance){
        return((int) Math.ceil(
                distance*ENCODER_PULSES_PER_REV/32
        ));
    }
    int degreesToTicks(double degrees){
        //actually gets pretty close to a perfect 90deg turn when testing
        return((int) Math.ceil(degrees* 1
        ));
    }
}
