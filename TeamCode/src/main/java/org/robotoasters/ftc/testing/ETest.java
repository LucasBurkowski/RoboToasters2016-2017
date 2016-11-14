/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.robotoasters.ftc.testing;

import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="ETest", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class ETest extends OpMode {
   SensorManager mSensorManager;
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private static final int ENCODER_PULSES_PER_REV = 2880;
    private static final double PI = 3.1415;

    DcMotor motorLeft1;
    DcMotor motorLeft2;
    DcMotor motorRight1;
    DcMotor motorRight2;
    protected float leftY;
    protected float rightY;
    protected float pSquared;
    int target = inchesToTicks(48);
    protected int distToTravel;

    //color stuff
    //ColorSensor sensorRGB;

    //Phone Sensor
    //Sensor gyroMag;
    float currentAngle = 0;

    /*
    static final int LED_CHANNEL = 5;
    private float hsvValues[] = {0F,0F,0F};
    final float values[]  = hsvValues;
    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

    boolean bPrevState = false;
    boolean bCurrState = false;
    boolean bLedOn = true;

    DeviceInterfaceModule cdim = hardwareMap.deviceInterfaceModule.get("dim");

    */

    int inchesToTicks(double distance){
        return((int) Math.ceil(
                distance*ENCODER_PULSES_PER_REV/32
        ));
    }
    int degreesToTicks(double degrees){
        return((int) Math.ceil(degrees* 1
        ));
    }

    @Override
    public void init() {
        //mSensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        //gyroMag = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);

        //RGB init
        //cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        //sensorRGB = hardwareMap.colorSensor.get("color");
        //cdim.setDigitalChannelState(LED_CHANNEL,bLedOn);




        telemetry.addData("Status", "Initialized");
        telemetry.update();
        motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight1 = hardwareMap.dcMotor.get("motorRight1");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");

        motorLeft1.setDirection(DcMotor.Direction.FORWARD);
        motorLeft2.setDirection(DcMotor.Direction.FORWARD);
        motorRight1.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);

        motorLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;
        motorLeft1.setMode(mode);
        motorLeft2.setMode(mode);
        motorRight1.setMode(mode);
        motorRight2.setMode(mode);

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        // leftMotor  = hardwareMap.dcMotor.get("left motor");
        // rightMotor = hardwareMap.dcMotor.get("right motor");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        distToTravel = inchesToTicks(3); //3 inches is just over 90/4 degrees
        telemetry.addData("Set", " at %7d", distToTravel);
        motorLeft1.setTargetPosition(distToTravel);
        motorLeft2.setTargetPosition(distToTravel);
        motorRight1.setTargetPosition(-1 * distToTravel);
        motorRight2.setTargetPosition(-1 * distToTravel);


        runtime.reset();

        mode = DcMotor.RunMode.RUN_TO_POSITION;
        motorLeft1.setMode(mode);
        motorLeft2.setMode(mode);
        motorRight1.setMode(mode);
        motorRight2.setMode(mode);
        motorLeft1.setTargetPosition(inchesToTicks(48));
        motorLeft2.setTargetPosition(inchesToTicks(48));
        motorRight1.setTargetPosition(inchesToTicks(48));
        motorRight2.setTargetPosition(inchesToTicks(48));
        runtime.reset();

    }
        //state = 0;
        // run until the end of the match (driver presses STOP)
        @Override
        public void loop() {
        //while (opModeIsActive() && motorLeft1.isBusy() && motorLeft2.isBusy() && motorRight1.isBusy() && motorRight2.isBusy()){
            //telemetry.addData("Status", "Run Time: %7d", state);//runtime.toString());
            // Send telemetry message to indicate successful Encoder reset
            /*
            telemetry.addData("leftM0tor",  " at %7d :%7d",
                    motorLeft1.getCurrentPosition(),
                    motorLeft2.getCurrentPosition());
            telemetry.addData("rightMotor",  " at %7d :%7d",
                    motorRight1.getCurrentPosition(),
                    motorRight2.getCurrentPosition());
            telemetry.update();
            */
            currentAngle = SensorManager.AXIS_Y;

            pControl(.0001, -target, motorRight1.getCurrentPosition(), motorRight1);
            pControl(.0001, -target, motorRight2.getCurrentPosition(), motorRight2);
            pControl(.0001, -target, motorLeft1.getCurrentPosition(), motorLeft1);
            pControl(.0001, -target, motorLeft2.getCurrentPosition(), motorLeft2);
                // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);
            /*
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", sensorRGB.alpha());
            telemetry.addData("Red  ", sensorRGB.red());
            telemetry.addData("Green", sensorRGB.green());
            telemetry.addData("Blue ", sensorRGB.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });
            */
        }

        private void pControl(double P, double setPoint, double currentPosition, DcMotor motor){
            double error =  setPoint - currentPosition;
            motor.setPower(P * error);
            telemetry.addData("Pout", " at %7d :%7d", (P * error));
        }
    }

