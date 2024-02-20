package org.firstinspires.ftc.teamcode.classes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous

public final class blueCloseLeftMalek extends LinearOpMode {
    // slides
    DcMotor slideL;
    DcMotor slideR;

    // arm base core
    DcMotor armBase;

    // gripper arm core
    DcMotor gripperArm;

    // gripper tilting servo
    Servo tilting;

    // grippers
    Servo gripperL;
    Servo gripperR;

    @Override
    public void runOpMode() throws InterruptedException {

        slideL = hardwareMap.get(DcMotor.class, "SlideL");
        slideR = hardwareMap.get(DcMotor.class, "SlideR");
        armBase = hardwareMap.get(DcMotor.class, "armBase");
        gripperArm = hardwareMap.get(DcMotor.class, "gripperArm");
        tilting = hardwareMap.get(Servo.class, "tilting");
        gripperL = hardwareMap.get(Servo.class, "gripperL");
        gripperR = hardwareMap.get(Servo.class, "gripperR");

        ((DcMotorEx) armBase).setVelocityPIDFCoefficients(40, 0.1, 0.1, 50);
        ((DcMotorEx) armBase).setPositionPIDFCoefficients(5);
        ((DcMotorEx) gripperArm).setVelocityPIDFCoefficients(40, 0.1, 0.1, 50);
        ((DcMotorEx) gripperArm).setPositionPIDFCoefficients(5);
        // reversed motors
        gripperL.setDirection(Servo.Direction.REVERSE);
        gripperArm.setDirection(DcMotorSimple.Direction.REVERSE);


        // stop and reset encoder
        slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gripperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gripperR.setPosition(1);
        gripperL.setPosition(1);

        waitForStart();
        Pose2d beginPose = new Pose2d(11, 60, 3 * Math.PI / 2);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


            GripperArm(540, 0.2);
            sleep(500);
            tilting.setPosition(0.8);


            Actions.runBlocking(
                    drive.actionBuilder(beginPose)

                            .lineToY(43)
                            .turnTo(-1.22173)
                            .waitSeconds(1)

                            .build());

            gripperR.setPosition(0);
            sleep(500);

            GripperArm(0, 0.2);
            ArmBase(230, 0.5);
            gripperR.setPosition(1);

            sleep(1000);

            tilting.setPosition(0.7);

            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(11, 43, -1.22173))

                            .turnTo(2 * Math.PI)
                            .lineToX(40)

                            .turnTo(3 * Math.PI / 2)

                            .lineToY(40)


                            .turnTo(2 * Math.PI)
                            .build());


//            sleep(1000);
            Open_Tilting();
            GripperArm(400, 0.2);
            sleep(2000);

//            Actions.runBlocking(trajRight(drive));


            gripperL.setPosition(0);
            sleep(500);
            gripperL.setPosition(1);

            GripperArm(0, 0.2);
            sleep(500);

            ArmBase(0, 0.5);

            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(42, 26, 2 * Math.PI))
                            .lineToX(50)
                            .turnTo(3 * Math.PI / 2)
                            .lineToY(60)
                            .build());


        }
    }

    private void ArmBase(int Ticks, double Power) {
        armBase.setTargetPosition(Ticks);
        armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armBase.setPower(Power);
    }


    private void GripperArm(int Ticks2, double Power2) {
        gripperArm.setTargetPosition(Ticks2);
        gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gripperArm.setPower(Power2);
    }

    private void Open_GripperL() {
        gripperR.setPosition(0);
        gripperL.setPosition(0);
        sleep(200);
    }


    private void Close_Tilting() {
        tilting.setPosition(0.4);
        sleep(200);
    }

    private void Open_Tilting() {
        tilting.setPosition(0.7);
        sleep(200);
    }


    private void Close_GripperL() {
        gripperR.setPosition(1);
        gripperL.setPosition(1);
        sleep(200);

    }

    public Action trajRight(MecanumDrive drive) {
        return drive.actionBuilder(new Pose2d(12, -62, Math.toRadians(-90)))
                .setReversed(true)
                // GOTO GROUND PIXEL
                .splineToConstantHeading(new Vector2d(15, -47), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(180)), Math.toRadians(0))
                .endTrajectory()
//                .stopAndAdd(motorActions.claw.release())
                .build();


    }
}