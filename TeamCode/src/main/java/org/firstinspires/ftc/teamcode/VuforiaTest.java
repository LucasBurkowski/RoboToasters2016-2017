package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.opengl.models.Teapot;


/**
 * Created by jacost63 on 9/28/2016.
 */
@TeleOp(name="VuforiaTest", group="Linear Opmode")
public class VuforiaTest extends LinearOpMode {




    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "ASWfaHf/////AAAAGZz1J5GuRkProXI9TAVR3P0qosjs6vYAwcHknPmJtBzpsaAvgM6kpgxVHcCMMWVC+61+gDUH9YcPEahwRAIZJPtuiGGxbAQToOO3xOab5i4mIntNLCguABtB7Dc/a4KjbbXDUT/I9dKqCr8flb1DtgfIUheG8GztponPLpeaFKa/Pk03UEixEnTLOcxdz6Y8gfdN4v01FMR7qtDfbsBpi2dz8X+9+SvzfTBkHX5EvmshVgo7fBTVmmB9z7AKtPO1v4lJTxNE/u4zpRIF79FR04/XjqpkMIU4HGg0AY8vmTAnq9qIjhS7FymbKAmbu4bfluZHXsmKEg6Gd9hNXm7spP+hnkNhe9eGdnDR3fTz00C0";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vf = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);

        VuforiaTrackables beacons = vf.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        waitForStart();
        
        beacons.activate();


        while(opModeIsActive()){
            for(VuforiaTrackable beac : beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
                if(pose!=null) {
                    VectorF translation = pose.getTranslation();
                    telemetry.addData(beac.getName() + "Translation", translation);
                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));
                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn); //for vertial phone. use .get(0), get(2) for horizonal
                }
            }
            telemetry.update();
        }
    }

}
