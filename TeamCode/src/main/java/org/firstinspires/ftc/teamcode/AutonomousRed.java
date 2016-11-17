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


/**
 * Created by team 8487/11735 on 9/28/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutonomousBlue", group="Opmode")
public class AutonomousRed extends OpMode {
    private static final int ENCODER_PULSES_PER_REV = 2880;
    private static final double PI = 3.1415;
    private static final double angleError = 5.0;
    private static final double optimalBeaconDistance = 2.0;

    DcMotor motorLeft1;
    DcMotor motorLeft2;
    DcMotor motorRight1;
    DcMotor motorRight2;
    VuforiaTrackable beac;
    double setpoint = 180;
    double error =0;
    double pValue = .1;
    VuforiaTrackables beacons;

    double degreesToTurn;

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

        motorLeft1 = hardwareMap.dcMotor.get("motorLeft1");
        motorLeft2 = hardwareMap.dcMotor.get("motorLeft2");
        motorRight1 = hardwareMap.dcMotor.get("motorRight1");
        motorRight2 = hardwareMap.dcMotor.get("motorRight2");
        motorLeft1.setDirection(DcMotor.Direction.REVERSE);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorRight1.setDirection(DcMotor.Direction.FORWARD);
        motorRight2.setDirection(DcMotor.Direction.FORWARD);
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
        beacons.get(3).setName("Gears");
    }


    @Override
    public void loop() {
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
        for(VuforiaTrackable beac : beacons) {
            if (pose != null) {
                VectorF translation = pose.getTranslation();
                telemetry.addData(beac.getName() + "Translation", translation);
                degreesToTurn = 180-Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                telemetry.addData(beac.getName() + "-Degrees", degreesToTurn); //for vertial phone. use .get(0), get(2) for horizonal
            }
        }

        switch(state){
            case START:
                switch(StartState){
                    case FWD1:
                        //fwd 2 ft
                        break;
                    case TURN:
                        //rotate 90deg
                        break;
                    case FWD2:
                        //forward
                        break;
                    default:
                        bleach();
                        break;
                }
                //if image detected
                state = StateMachine.VUFORIA_NAV;
                break;
            case VUFORIA_NAV:
                switch(VuforiaState){
                    case TURN1:
                        //Turn 90-theta
                        VuforiaState = SMVuforia.FWD;
                        break;
                    case FWD:
                        //Go fwd x inches
                        VuforiaState = SMVuforia.TURN2;
                        break;
                    case TURN2:
                        //turn -90deg
                        VuforiaState = SMVuforia.RECHECK;
                        break;
                    case RECHECK:
                        if(degreesToTurn <= angleError){
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
                //Forward
                charge();
                // if distance = optimalBeaconDistance
                state = StateMachine.BOB;
                // if distance < optimalBeaconDistance
                state = StateMachine.BACKUP;
                // if theta > angleError
                if(degreesToTurn > angleError){
                    state = StateMachine.VUFORIA_NAV;
                }
                break;
            case BOB:
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
        motorLeft1.setPower(0);
        motorLeft2.setPower(0);
        motorRight1.setPower(0);
        motorRight2.setPower(0);
    }
    void charge(){
        motorLeft1.setPower(1);
        motorLeft2.setPower(1);
        motorRight1.setPower(-1);
        motorRight2.setPower(-1);
    }
    void retreat(){
        motorLeft1.setPower(-1);
        motorLeft2.setPower(-1);
        motorRight1.setPower(1);
        motorRight2.setPower(1);
    }

}


