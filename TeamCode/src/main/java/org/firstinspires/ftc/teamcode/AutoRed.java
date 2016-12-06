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

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.robotoasters.ftc.helper.BeaconHandler;

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
    DeviceInterfaceModule cdim;
    DcMotor motorLeft1;
    DcMotor motorLeft2;
    DcMotor motorRight1;
    DcMotor motorRight2;
    BeaconHandler beaconPush;
    Orientation angles;
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
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
        turnRightAmount(90);
        while (opModeIsActive()){
            telemetry.addData("gyro",  " at %.3f",
                    getAngle());
            telemetry.update();
        }
    }

    public void turnRightAmount(float degrees){
        resetEncoders();
        //runWithoutEncoders();
        turnRight(1);
        telemetry.addData("gyro",  " at %.3f",
                getAngle());
        while(getAngle() < degrees){
            telemetry.addData("gyro",  " at %.3f", getAngle());
        }
        turnRight(0);
        runWithEncoders();
    }

    public void turnLeftAmount(float degrees){
        resetEncoders();
        runWithoutEncoders();
        turnLeft(1);
        while(getAngle() > degrees){
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
            telemetry.addData("gyro",  " at %.3f",
                    getAngle());
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

    private void runWithoutEncoders(){
        motorLeft1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private int inchesToTicks(double distance){
        return((int) Math.ceil(
                distance*2880/32
        ));
    }

    private int degreesToTicks(double degrees){
        return((int) Math.ceil(inchesToTicks(Math.toRadians(degrees) * (16/2) //arc length = angle * radius
        )));
    }

    private float getAngle(){
        float currentHeading = Math.abs(imu.getAngularOrientation().firstAngle);
        if (currentHeading > 180)
            currentHeading = currentHeading - 360;
        if (currentHeading < 180)
            currentHeading = currentHeading + 360;
        return currentHeading;
    }



}