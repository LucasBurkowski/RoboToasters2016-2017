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

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

@Autonomous(name="Autonomous_TEST!", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class AutoDriveTest extends LinearOpMode {
    DeviceInterfaceModule cdim;
    DcMotor motorLeft1;
    DcMotor motorLeft2;
    DcMotor motorRight1;
    DcMotor motorRight2;
    BeaconHandler beaconPush;
    BNO055IMU imu;
    double turnKp = 0.026;//0.04 //0.025
    double driveKp = 0.025;//0.015 //0.025
    double currentTrnTgt = 0;

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
        motorLeft1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        beaconPush = new BeaconHandler(hardwareMap);
        /*
        waitForStart();
        driveAmount(5);
        turnAmount(-30);
        driveAmount(50);
        turnAmount(-90);
        */
        driveWhileTurning(78, -90);
        driveAmount(8);//11  orange //8 blue
        beaconPush.pressBeacon(beaconPush.RED);
        Thread.sleep(2000);
        driveAmount(3);
        driveAmount(-5);
        beaconPush.resetPaddles();
        turnAmount(0);
        driveAmount(45);//45 orange //40 blue
        turnAmount(-90);
        driveAmount(5);
        beaconPush.pressBeacon(beaconPush.RED);
        Thread.sleep(2000);
        driveAmount(3);
        driveAmount(-5);
        beaconPush.resetPaddles();

        while (opModeIsActive()){
            telemetry.addData("gyro",  " at %.3f",
                    getAngle());
            telemetry.update();
        }
    }

    public void turnAmount(float degrees){
        resetEncoders();
        while((getAngle() < degrees - 1 || getAngle() > degrees + 1) && opModeIsActive()){
            arcadeDrive(0, Ploop(degrees, getAngle(), turnKp));
            telemetry.addData("gyro",  " at %.3f",
                    getAngle());
            telemetry.update();
        }
        arcadeDrive(0, 0);
        currentTrnTgt = getAngle();
    }

    public void driveWhileTurning(int distance, int degrees){
        int dist = inchesToTicks(distance);
        while(motorRight2.getCurrentPosition() < dist && (getAngle() < degrees - 1 || getAngle() > degrees + 1) && opModeIsActive()){
            arcadeDrive(Ploop(dist, motorRight2.getCurrentPosition(), driveKp), Ploop(degrees, getAngle(), turnKp));
        }
    }

    public void driveAmount(int distance){
        int dist = inchesToTicks(distance);
        resetEncoders();
        while((motorRight2.getCurrentPosition() > dist + 0.5 || motorRight2.getCurrentPosition() < dist - 0.5) && opModeIsActive()){
            arcadeDrive(Ploop(dist, motorRight2.getCurrentPosition(), driveKp), Ploop(currentTrnTgt, getAngle(), turnKp));
            telemetry.addData("motorRight2",  " at %7d",motorRight2.getCurrentPosition());
            telemetry.update();
        }
        arcadeDrive(0, 0);
    }

    private void resetEncoders(){
        motorRight1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private int inchesToTicks(double distance){
        return((int) Math.ceil(
                distance*2880/32
        ));
    }

    private double Ploop(double target, double current, double Kp) {
        double error;
        error = current - target;
        return Math.abs(error * Kp);
    }

    private float getAngle(){
        float currentHeading = AngleUnit.DEGREES.normalize(imu.getAngularOrientation().firstAngle);
        return currentHeading;
    }

    private void arcadeDrive(double frwdBck, double lftRgt){
        double leftMotors = frwdBck - lftRgt;
        double rightMotors = lftRgt + frwdBck;
        motorLeft1.setPower(leftMotors);
        motorLeft2.setPower(leftMotors);
        motorRight1.setPower(rightMotors);
        motorRight2.setPower(rightMotors);
    }

}
