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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name="Template: Linear OpMode", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class TemplateOpMode_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private static final int ENCODER_PULSES_PER_REV = 280;
    private static final double PI = 3.1415;

    DcMotor motorLeft1;
    DcMotor motorLeft2;
    DcMotor motorRight1;
    DcMotor motorRight2;
    protected float leftY;
    protected float rightY;
    protected float pSquared;

    protected int distToTravel;
    int state;
    DcMotor.RunMode mode;

    int inchesToTicks(double distance){
        return((int)java.lang.Math.ceil(
                (distance * 70) / PI
        ));
    }

    @Override
    public void runOpMode() throws InterruptedException {
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
        idle();

        mode = DcMotor.RunMode.RUN_USING_ENCODER;
        motorLeft1.setMode(mode);
        motorLeft2.setMode(mode);
        motorRight1.setMode(mode);
        motorRight2.setMode(mode);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("leftMotor",  "Starting at %7d :%7d",
                motorLeft1.getCurrentPosition(),
                motorLeft2.getCurrentPosition());
        telemetry.update();
        telemetry.addData("RightMotor",  "Starting at %7d :%7d",
                motorRight1.getCurrentPosition(),
                motorRight2.getCurrentPosition());
        telemetry.update();


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

        distToTravel = 280;//inchesToTicks(12);
        telemetry.addData("Set",  " at %7d", distToTravel);
        telemetry.update();
        motorLeft1.setTargetPosition(distToTravel);
        motorLeft2.setTargetPosition(distToTravel);
        motorRight1.setTargetPosition(distToTravel);
        motorRight2.setTargetPosition(distToTravel);

        telemetry.addData("leftMotor",  "Starting at %7d :%7d",
                motorLeft1.getCurrentPosition(),
                motorLeft2.getCurrentPosition());
        telemetry.addData("RightMotor",  "Starting at %7d :%7d",
                motorRight1.getCurrentPosition(),
                motorRight2.getCurrentPosition());

        telemetry.addData("Target",  " at %7d %7d %7d %7d",
                            motorLeft1.getTargetPosition(),
                            motorLeft2.getTargetPosition(),
                            motorRight1.getTargetPosition(),
                            motorRight2.getTargetPosition()
                );
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        motorLeft1.setTargetPosition(distToTravel);
        motorLeft2.setTargetPosition(distToTravel);
        motorRight1.setTargetPosition(distToTravel);
        motorRight2.setTargetPosition(distToTravel);
        mode = DcMotor.RunMode.RUN_TO_POSITION;
        motorLeft1.setMode(mode);
        motorLeft2.setMode(mode);
        motorRight1.setMode(mode);
        motorRight2.setMode(mode);
        runtime.reset();
        motorLeft1.setPower(.10);
        motorLeft2.setPower(.10);
        motorRight1.setPower(.10);
        motorRight2.setPower(.10);
        telemetry.addData("leftMotor",  "Starting at %7d :%7d",
                motorLeft1.getCurrentPosition(),
                motorLeft2.getCurrentPosition());
        telemetry.addData("RightMotor",  "Starting at %7d :%7d",
                motorRight1.getCurrentPosition(),
                motorRight2.getCurrentPosition());

        telemetry.addData("Pwr",  " at %7f %7f %7f %7f",
                motorLeft1.getPower(),
                motorLeft2.getPower(),
                motorRight1.getPower(),
                motorRight2.getPower()
        );
        telemetry.update();
        state = 0;
        // run until the end of the match (driver presses STOP)
        //while (opModeIsActive() && (runtime.seconds() < 3 ) && (motorLeft1.isBusy() && motorRight1.isBusy() && motorLeft2.isBusy() && motorRight2.isBusy()) ) {
        while(motorRight1.isBusy() ){
            telemetry.addData("RightMotor1",  "at %7d, %7d",
                    motorRight1.getCurrentPosition(),
                    state++
                    );
            telemetry.update();
            idle();
        }
        /*while (opModeIsActive() && (runtime.seconds() < 3) &&
                (motorLeft1.isBusy() && motorLeft2.isBusy() && motorRight1.isBusy() && motorRight2.isBusy())
                ) {
            //telemetry.addData("Status", "Run Time: %7d", state);//runtime.toString());
            // Send telemetry message to indicate successful Encoder reset
            telemetry.addData("l",  " at %7d :%7d",
                    motorLeft1.getCurrentPosition(),
                    motorLeft2.getCurrentPosition());
            telemetry.update();
            telemetry.addData("rightMotor",  " at %7d :%7d",
                    motorRight1.getCurrentPosition(),
                    motorRight2.getCurrentPosition());
            telemetry.update();

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);
            state++;
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
        */
        motorLeft1.setPower(0);
        motorLeft2.setPower(0);
        motorRight1.setPower(0);
        motorRight2.setPower(0);
        mode = DcMotor.RunMode.RUN_USING_ENCODER;
        motorLeft1.setMode(mode);
        motorLeft2.setMode(mode);
        motorRight1.setMode(mode);
        motorRight2.setMode(mode);

    }
}
