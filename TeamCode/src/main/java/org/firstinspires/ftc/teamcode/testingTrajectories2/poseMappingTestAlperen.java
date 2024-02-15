package org.firstinspires.ftc.teamcode.testingTrajectories2;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous
public class poseMappingTestAlperen extends LinearOpMode {

    DcMotor gripperMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        gripperMotor = hardwareMap.dcMotor.get("urName");

        gripperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        Pose2d beginPose = new Pose2d(12, -60, Math.PI / 2);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .lineToY(-47)
                            .build());

        }

        gripperMotor.setTargetPosition(100);
        gripperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gripperMotor.setPower(0.2);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

//            Actions.runBlocking(
//                    drive.actionBuilder(beginPose,
//                                    pose -> new Pose2dDual<>(pose.position.x,
//                                            pose.position.y.unaryMinus(),
//                                            pose.heading.inverse()))
//                            .lineToY(-60)
//                            .build());

        }

    }
}
