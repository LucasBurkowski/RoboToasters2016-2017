package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;


/**
 * Created by team 8487/11735 on 9/28/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutonomousBlue", group="Opmode")
public class AutonomousRed extends OpMode {
    private static final int ENCODER_PULSES_PER_REV = 2880;
    private static final double PI = 3.1415;
    private static final double angleError = 5.0;
    private static final double optimalBeaconDistance = 2.0;
    private static final double MM_TO_INCH = 25.4;


    DcMotor[] motors;

    VuforiaTrackable beac;
    int currentTargetBeacon;
    double setpoint = 180;
    double error =0;
    double pValue = .1;
    VuforiaTrackables beacons;
    boolean[] targetAcquired;
    double[] degreesToTurn;
    double lateralDistance;
    double distanceToTarget;


    enum StateMachine {START, VUFORIA_NAV, ATTACK, BOB, BACKUP};
    enum SMStart {FWD1,TURN,FWD2};
    enum SMVuforia {TURN1, FWD, TURN2,RECHECK};

    StateMachine state;
    SMStart StartState;
    SMVuforia VuforiaState;

    @Override
    public void init(){
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        VuforiaLocalizer vf = ClassFactory.createVuforiaLocalizer(params);

        motors = new DcMotor[4];
        motors[0] = hardwareMap.dcMotor.get("motorLeft1");
        motors[1] = hardwareMap.dcMotor.get("motorLeft2");
        motors[2] = hardwareMap.dcMotor.get("motorRight1");
        motors[3] = hardwareMap.dcMotor.get("motorRight2");

        motors[0].setDirection(DcMotor.Direction.REVERSE);
        motors[1].setDirection(DcMotor.Direction.REVERSE);
        motors[2].setDirection(DcMotor.Direction.FORWARD);
        motors[3].setDirection(DcMotor.Direction.FORWARD);

        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "ARqxqWv/////AAAAGcgaMQP8ikedq3w/jrpvXskoATL6NREllrV7IGCIHz9zGUR4f3DXxaQVY4vo2j6jFr9KdWPrUnHYPOL5bXud1WEUfgAVIj9j9fIsj7e1Bl3levbPmiCBWvLFYWJUPY59zF/NLtRlebCK6/APqWS/JcOtI3cKNFZDhCIvE2cxAN7dc7TPmglO/451FRo/4oQTTgKA+JkBXtVCEOnNdvsEp6enEnfwA4nKGiFk2O/k/gTDr7kruLrkbf/utPvsqe4rw+XszYM21vF0PBtYynKYPiWsOT7U75XeLvCDma2JirBE0U3gMtkvoeELnztWtNtXE1J5dW6xyJu9yXGleLpwcCO80FV0RJIWIu0GpQF/RZZq";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);
        beacons = vf.loadTrackablesFromAsset("FTC_2016-17");
        beacons.activate();
        //0
        //1  is height
        //2   is distance
        //bottom is degrees going negative

        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");//here's our target
        currentTargetBeacon = 3;
        targetAcquired = new boolean[4];
        degreesToTurn = new double[4];
        for(int i = 0;i<4;i++){
            targetAcquired[i] = FALSE;
            degreesToTurn[i] = 0;
        }

        encoderReset();
    }


    @Override
    public void loop() {
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
        for(VuforiaTrackable beac : beacons) {
            if (pose != null) {
                VectorF translation = pose.getTranslation();
                telemetry.addData(beac.getName() + "Translation", translation);
                double dtt = 180-Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                telemetry.addData(beac.getName() + "-Degrees", degreesToTurn); //for vertial phone. use .get(0), get(2) for horizonal

                //if we found a new valid beacon
                if(beac.getName().equals("Wheels")) {
                    targetAcquired[0] = TRUE;
                    degreesToTurn[0] = dtt;
                }
                else if(beac.getName().equals("Tools")){
                    targetAcquired[1] = TRUE;
                    degreesToTurn[1] = dtt;
                }
                else if(beac.getName().equals("Lego")){
                    targetAcquired[2] = TRUE;
                    degreesToTurn[2] = dtt;
                }
                else if(beac.getName().equals("Gears")){
                    targetAcquired[3] = TRUE;
                    degreesToTurn[3] = dtt;
                    lateralDistance = translation.get(2)/MM_TO_INCH;
                    distanceToTarget = Math.abs(translation.get(1)/MM_TO_INCH);
                }
            }
        }

        switch(state){
            case START:
                telemetry.addLine("SM: Start");
                switch(StartState){
                    case FWD1:
                        //fwd 2 ft
                        if(safeDrive(inchesToTicks(24.0))){
                            bleach();
                            encoderReset();
                            StartState = SMStart.TURN;
                        }
                        break;
                    case TURN:
                        //turn 90deg to face the camera towards beacons
                        if(encoderTurn(degreesToTicks(90))){
                            bleach();
                            encoderReset();
                            StartState = SMStart.FWD2;
                        }
                        break;
                    case FWD2:
                        //forward
                        charge();
                        //if image detected
                        if(targetAcquired[currentTargetBeacon]){
                            bleach();
                            encoderReset();
                            state = StateMachine.VUFORIA_NAV;
                            VuforiaState = SMVuforia.TURN1;
                        }
                        break;
                    default:
                        bleach();
                        break;
                }
                break;
            case VUFORIA_NAV:
                telemetry.addLine("SM: VuforiaNav");
                switch(VuforiaState){
                    case TURN1:
                        //Turn 90-theta
                        if(safeTurn(90-degreesToTurn[currentTargetBeacon])){
                            bleach();
                            encoderReset();
                            VuforiaState = SMVuforia.FWD;
                        }
                        break;
                    case FWD:
                        //Go fwd x inches
                        if(safeDrive(inchesToTicks(Math.abs(lateralDistance)))){
                            bleach();
                            encoderReset();
                            VuforiaState = SMVuforia.TURN2;
                        }
                        break;
                    case TURN2:
                        //turn -90deg
                        if(encoderTurn(degreesToTicks(-90))){
                            bleach();
                            encoderReset();
                            VuforiaState = SMVuforia.RECHECK;
                        }
                        break;
                    case RECHECK:
                        if(degreesToTurn[currentTargetBeacon] <= angleError){
                            state = StateMachine.ATTACK;
                            VuforiaState = SMVuforia.TURN1;
                        }else{
                            VuforiaState = SMVuforia.TURN1;
                        }
                        break;
                    default:
                        bleach();
                        break;
                }
                break;
            case ATTACK:
                telemetry.addLine("SM: Attack");
                //Forward
                charge();
                // if distance <= optimalBeaconDistance
                if(distanceToTarget <= optimalBeaconDistance) {
                    bleach();
                    encoderReset();
                    state = StateMachine.BOB;
                }
                // if distance < optimalBeaconDistance
                // don't think we'll need this right now state = StateMachine.BACKUP;

                // if theta > angleError
                if(degreesToTurn[currentTargetBeacon] > angleError){
                    bleach();
                    encoderReset();
                    state = StateMachine.VUFORIA_NAV;
                }
                break;
            case BOB:
                telemetry.addLine("SM: BOB!");
                //Lucus stuff
                break;
            case BACKUP:
                retreat();
                //if distance > optimalBeaconDistance
                state = StateMachine.VUFORIA_NAV;
                break;
            default:
                bleach();
                break;
        }

        //clear targets, must lock every loop
        for(int i = 0;i<4;i++){
            targetAcquired[i] = FALSE;
        }
        telemetry.update();
    }//end loop


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

    void bleach(){
        for(int i = 0; i<4;i++){
            motors[i].setPower(0);
        }
    }
    void charge(){
        for(int i = 0; i<4;i++){
            motors[i].setPower(1);
        }
    }
    void retreat(){
        for(int i = 0; i<4;i++){
            motors[i].setPower(-1);
        }
    }

    void encoderReset(){
        DcMotor.RunMode mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        for(int i = 0; i<4;i++){
            motors[i].setMode(mode);
        }
        mode = DcMotor.RunMode.RUN_USING_ENCODER;
        for(int i = 0; i<4;i++){
            motors[i].setMode(mode);
        }
    }
    /*
    *safeDrive
    * Moves the desired wheel tick using encoders
    * Tries to account for lost encoders
    * smooths the encoder values with simple averaging
    * returns true when we hit the target tick count
    * REQUIRES: a reset encoder before first call
    */
    boolean safeDrive(int targetTicks){
        double tTicks = 0;
        int nWheels = 0;
        for(int i = 0; i<4;i++){
            if(motors[i].getCurrentPosition() != 0){
                tTicks += motors[i].getCurrentPosition();
                nWheels++;
            }
        }
        tTicks = tTicks/nWheels; //average over all the working wheels
        if(targetTicks >0) {//moving forward
            if(tTicks >= targetTicks){
                return TRUE;
            }else{
                charge();
            }
        }else{//moving backwards
            if(tTicks <= targetTicks) {
                return TRUE;
            }else{
                retreat();
            }
        }
        return FALSE;
    }
/**
 * Turn using encoders
 * Filters out bad encoders and smooths
 * I might have flipped left and right :<
 * REQUIRES: a reset encoder before first call
* */
    boolean encoderTurn(int targetTicks){
        double tTicks = 0;
        int nWheels = 0;
        for(int i = 0;i<4;i++){
            if(motors[i].getCurrentPosition() != 0){
                tTicks += Math.abs(motors[i].getCurrentPosition());
                nWheels++;
            }
        }
        tTicks = tTicks/nWheels;
        if(targetTicks > 0){
            if(tTicks >= targetTicks){
                return TRUE;
            }else{
                motors[0].setPower(-1);
                motors[1].setPower(-1);
                motors[2].setPower(1);
                motors[3].setPower(1);
            }
        }else if(targetTicks < 0){
            if((-1*tTicks) <=targetTicks){
                return TRUE;
            }else{
                motors[0].setPower(1);
                motors[1].setPower(1);
                motors[2].setPower(-1);
                motors[3].setPower(-1);
            }
        }
        return FALSE;
    }
    /*
    * Turn towards the images with a changing angle
    * does not rely on encoders.
    * */
    boolean safeTurn(double angle){
        int ticks = degreesToTicks(angle);
        if(angle > angleError){//need to turn right
            motors[0].setPower(-1);
            motors[1].setPower(-1);
            motors[2].setPower(1);
            motors[3].setPower(1);
            return FALSE;
        }else if(angle < (-1*angleError)){//need to turn left
            motors[0].setPower(1);
            motors[1].setPower(1);
            motors[2].setPower(-1);
            motors[3].setPower(-1);
            return FALSE;
        }else{
            return TRUE;
        }
    }

    private void pControl(double P, double setPoint, double currentPosition, DcMotor motor){
        double error =  setPoint - currentPosition;
        motor.setPower(P * error);
        telemetry.addData("Pout", " at %7d :%7d", (P * error));
    }

}


