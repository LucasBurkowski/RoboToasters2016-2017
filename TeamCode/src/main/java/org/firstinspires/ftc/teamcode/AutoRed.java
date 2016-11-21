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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.robotoasters.ftc.helper.BeaconHandler;
import org.robotoasters.ftc.helper.NavigationHandler;

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

@Autonomous(name="Autonomous_Red", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class AutoRed extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    NavigationHandler nav;
    DcMotor motorLeft1;
    DcMotor motorLeft2;
    DcMotor motorRight1;
    DcMotor motorRight2;
    BeaconHandler beaconPush;

    @Override
    public void runOpMode() throws InterruptedException {
        nav = new NavigationHandler(hardwareMap);
        motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight1 = hardwareMap.dcMotor.get("motorRight1");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");
        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorRight1.setDirection(DcMotor.Direction.FORWARD);
        motorRight2.setDirection(DcMotor.Direction.FORWARD);
        motorLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        beaconPush = new BeaconHandler(hardwareMap);
        waitForStart();
        driveStraightAmount(24, 0.5);
        turnLeftAmount(45);
        driveStraightAmount(20, 0.5);
        turnLeftAmount(60);
        driveStraightAmount(15, 0.4);
        beaconPush.pressBeacon(beaconPush.BLUE);
        driveStraightAmount(4, 0.5);

        while (opModeIsActive()){

        }
    }

    public void turnRightAmount(int degrees){
        int turnDist = degreesToTicks(degrees);
        resetEncoders();
        runToPos();
        setLeftTarget(-turnDist);
        setRightTarget(turnDist);
        turnRight(1);
        while(motorLeft1.isBusy() && motorRight1.isBusy()){
            telemetry.addData("motorRight",  " at %7d",
                    motorLeft1.getCurrentPosition());
            telemetry.addData("target", " %7d", motorRight2.getTargetPosition());
            telemetry.update();
        }
        turnRight(0);
        runWithEncoders();
    }

    public void turnLeftAmount(int degrees){
        int turnDist = degreesToTicks(degrees);
        resetEncoders();
        runToPos();
        setLeftTarget(turnDist);
        setRightTarget(-turnDist);
        turnLeft(1);
        while(motorLeft1.isBusy()&& motorRight1.isBusy()){
        }
        turnLeft(0);
        runWithEncoders();
    }

    public void driveStraightAmount(int distance, double power){
        int dist = inchesToTicks(distance);
        resetEncoders();
        setRightTarget(dist);
        setLeftTarget(dist);
        runToPos();
        driveStraight(power);
        while(motorLeft1.isBusy()&& opModeIsActive()){
            telemetry.addData("motorRight",  " at %7d",
                    motorLeft1.getCurrentPosition());
            telemetry.addData("target", " %7d", motorRight2.getTargetPosition());
            telemetry.update();
        }
        driveStraight(0);
        runWithEncoders();
    }

    private void driveStraight(double power){
        motorLeft1.setPower(power);
        motorLeft2.setPower(power);
        motorRight1.setPower(power);
        motorRight2.setPower(power);
    }

    private void turnRight(double power){
        motorLeft1.setPower(power);
        motorLeft2.setPower(power);
        motorRight1.setPower(-power);
        motorRight2.setPower(-power);
    }

    private void turnLeft(double power){
        motorLeft1.setPower(-power);
        motorLeft2.setPower(-power);
        motorRight1.setPower(power);
        motorRight2.setPower(power);
    }

    private void setLeftTarget(int distance){
        motorLeft1.setTargetPosition(distance);
        motorLeft2.setTargetPosition(distance);
    }

    private void setRightTarget(int distance){
        motorRight1.setTargetPosition(distance);
        motorRight2.setTargetPosition(distance);
    }

    private void resetEncoders(){
        motorRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void runWithEncoders(){
        motorRight1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void runToPos(){
        motorLeft1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private int inchesToTicks(double distance){
        return((int) Math.ceil(
                distance*2880/32
        ));
    }

    private int degreesToTicks(double degrees){
        return((int) Math.ceil(inchesToTicks(Math.toRadians(degrees) * (16/2) //arc length = angle * radius
        ));
    }
}
