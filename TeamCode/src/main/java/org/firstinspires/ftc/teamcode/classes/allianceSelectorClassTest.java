package org.firstinspires.ftc.teamcode.classes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class allianceSelectorClassTest {

    public String[] autoSelector(){
        // Auto Selector
        String alliance = "BLUE_ALLIANCE";
        String side = "SIDE_CLOSE";

//        while (!opModeIsActive() && !isStopRequested()){
            if (gamepad1.x){
                alliance = "BLUE_ALLIANCE";
            }else if (gamepad1.circle){
                alliance = "RED_ALLIANCE";
            }
            telemetry.addData("Select Alliance (Gamepad1 Cross = Blue, Gamepad1 Circle = Red)", "");
            telemetry.addData("Current Alliance Selected : ", alliance.toUpperCase());
            telemetry.addData("", "");

            if (gamepad1.dpad_left){
                side = "SIDE_FAR";
            }else if (gamepad1.dpad_right){
                side = "SIDE_CLOSE";
            }
            telemetry.addData("Select Side (Gamepad1 DPad left = Far, Gamepad1 DPad right = Close)", "");
            telemetry.addData("Current Side Selected : ", side.toUpperCase());
            telemetry.addData("", "");

            telemetry.update();
//        }

        return new String[] {alliance, side};

    }
}
