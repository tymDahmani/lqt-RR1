package org.firstinspires.ftc.teamcode.reallyFinalCodesHere;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.classes.blobDetectionTest;

public class finalFinalFinal extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        int side = 0;
        String curAlliance = null;
        blobDetectionTest detector;


        while (!opModeIsActive() && !isStopRequested()){

            if (gamepad1.cross){
                curAlliance = "blue";
            }else if (gamepad1.circle){
                curAlliance = "red";
            }
            blobDetectionTest.setAlliancePipe(curAlliance);
            telemetry.addData("Select Alliance (Gamepad1 cross = Blue, Gamepad1 circle = Red)", "");
            telemetry.addData("Current Alliance Selected : ", curAlliance.toUpperCase());
            telemetry.update();

            if (gamepad1.dpad_left){
                side = 1;
            }else if (gamepad1.dpad_right){
                side = 2;
            }
            telemetry.addData("Select Side (Gamepad1 DPad left = Far, Gamepad1 DPad right = Close)", "");
            telemetry.addData("Current Side Selected : ", side);
            telemetry.addData("", "");

            telemetry.update();

        }

        // here will go our class for each alliance and side (far and left)


    }
}
