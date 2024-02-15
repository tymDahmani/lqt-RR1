package org.firstinspires.ftc.teamcode.testingTrajectories2;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "iliasManual (Blocks to Java)")
public class iliasManual extends LinearOpMode {

  private IMU imu_IMU;
  private DcMotor armBase;
  private DcMotor gripperArm;
  private DcMotor rightFront;
  private DcMotor rightBack;
  private DcMotor leftFront;
  private DcMotor leftBack;
  private Servo gripperL;
  private DcMotor SlideL;
  private DcMotor SlideR;
  private Servo gripperR;
  private Servo tilting;

  double multiplier;
  YawPitchRollAngles _7ByawPitchRollAnglesVariable_7D;
  double rotX;
  float x;
  float rx;
  double rotY;
  float y;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int leftDpad;
    int rightDpad;
    int HangingVariable;

    imu_IMU = hardwareMap.get(IMU.class, "imu");
    armBase = hardwareMap.get(DcMotor.class, "armBase");
    gripperArm = hardwareMap.get(DcMotor.class, "gripperArm");
    rightFront = hardwareMap.get(DcMotor.class, "rightFront");
    rightBack = hardwareMap.get(DcMotor.class, "rightBack");
    leftFront = hardwareMap.get(DcMotor.class, "leftFront");
    leftBack = hardwareMap.get(DcMotor.class, "leftBack");
    gripperL = hardwareMap.get(Servo.class, "gripperL");
    SlideL = hardwareMap.get(DcMotor.class, "SlideL");
    SlideR = hardwareMap.get(DcMotor.class, "SlideR");
    gripperR = hardwareMap.get(Servo.class, "gripperR");
    tilting = hardwareMap.get(Servo.class, "tilting");

    // Initializes the IMU with non-default settings. To use this block,
    // plug one of the "new IMU.Parameters" blocks into the parameters socket.
    // Creates a Parameters object for use with an IMU in a REV Robotics Control Hub or
    // Expansion Hub, specifying the hub's orientation on the robot via the direction that
    // the REV Robotics logo is facing and the direction that the USB ports are facing.
    imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
    // Reverse one of the drive motors.
    ((DcMotorEx) armBase).setVelocityPIDFCoefficients(40, 2, 0, 25);
    ((DcMotorEx) armBase).setPositionPIDFCoefficients(5);
    ((DcMotorEx) gripperArm).setVelocityPIDFCoefficients(40, 1, 1, 25);
    ((DcMotorEx) gripperArm).setPositionPIDFCoefficients(5);
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    rightFront.setDirection(DcMotor.Direction.REVERSE);
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    rightBack.setDirection(DcMotor.Direction.REVERSE);
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    leftFront.setDirection(DcMotor.Direction.FORWARD);
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    leftBack.setDirection(DcMotor.Direction.FORWARD);
    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    gripperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armBase.setDirection(DcMotor.Direction.FORWARD);
    gripperArm.setDirection(DcMotor.Direction.REVERSE);
    gripperL.setDirection(Servo.Direction.REVERSE);
    SlideL.setDirection(DcMotor.Direction.REVERSE);
    SlideR.setDirection(DcMotor.Direction.REVERSE);
    leftDpad = 1;
    rightDpad = 1;
    HangingVariable = 1;
    waitForStart();
    if (opModeIsActive()) {
      imu_IMU.resetYaw();
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        _7ByawPitchRollAnglesVariable_7D = imu_IMU.getRobotYawPitchRollAngles();
        if (gamepad1.right_bumper) {
          multiplier = 0.5;
        } else {
          multiplier = 1;
        }
        gamepad();
        setPower();
        // Pressing Right Bumper will Reset the IMU for Field Centric
        if (gamepad1.options) {
          imu_IMU.resetYaw();
        }
        data();
        if (gamepad1.left_stick_button) {
          gripperR.setPosition(1);
          gripperL.setPosition(1);
          tilting.setPosition(1);
          rightDpad = 1;
          leftDpad = 1;
          GripperArm(0, 0.1);
          ArmBase(130, 0.8);
          sleep(500);
        }
        if (rightDpad == 1 && gamepad1.dpad_right) {
          gripperR.setPosition(0);
          sleep(500);
          rightDpad = 0;
        }
        if (rightDpad == 0 && gamepad1.dpad_right) {
          gripperR.setPosition(1);
          sleep(500);
          rightDpad = 1;
        }
        if (leftDpad == 1 && gamepad1.dpad_left) {
          gripperL.setPosition(0);
          sleep(500);
          leftDpad = 0;
        }
        if (leftDpad == 0 && gamepad1.dpad_left) {
          gripperL.setPosition(1);
          sleep(500);
          leftDpad = 1;
        }
        if (gamepad1.dpad_up) {
          gripperR.setPosition(1);
          gripperL.setPosition(1);
          sleep(500);
          tilting.setPosition(0.3);
          GripperArm(200, 0.2);
          ArmBase(20, 0.8);
          rightDpad = 1;
          leftDpad = 1;
        }
        if (gamepad1.dpad_down) {
          GripperArm(540, 0.2);
          ArmBase(20, 0.8);
          sleep(1000);
          gripperR.setPosition(0);
          gripperL.setPosition(0);
          tilting.setPosition(1);
          rightDpad = 0;
          leftDpad = 0;
        }
        if (gamepad1.cross) {
          gripperR.setPosition(1);
          gripperL.setPosition(1);
          sleep(500);
          tilting.setPosition(1);
          GripperArm(400, 0.2);
          ArmBase(20, 0.8);
        }
        if (gamepad1.circle) {
          gripperR.setPosition(1);
          gripperL.setPosition(1);
          sleep(500);
          tilting.setPosition(1);
          GripperArm(380, 0.2);
          ArmBase(500, 0.8);
        }
        if (gamepad1.triangle) {
          gripperR.setPosition(1);
          gripperL.setPosition(1);
          sleep(500);
          tilting.setPosition(1);
          GripperArm(450, 0.2);
          ArmBase(800, 0.8);
        }
        if (gamepad1.square) {
          gripperR.setPosition(1);
          gripperL.setPosition(1);
          sleep(500);
          tilting.setPosition(0.1);
          GripperArm(520, 0.2);
          ArmBase(2000, 0.8);
        }
        if (HangingVariable == 1 && gamepad1.guide) {
          ArmBase(1500, 1);
          sleep(1000);
          Slides(2500, 2000);
          HangingVariable = 0;
        }
        if (HangingVariable == 0 && gamepad1.guide) {
          Slides(100, 2000);
          sleep(1000);
          ArmBase(800, 1);
          HangingVariable = 1;
        }
      }
    }
  }

  /**
   * Describe this function...
   */
  private void gamepad() {
    double theta;

    // Use left stick to drive and right stick to turn
    // You may have to negate the sticks. When you
    // negate a stick, negate all other instances of the stick
    x = gamepad1.left_stick_x;
    y = -gamepad1.left_stick_y;
    rx = gamepad1.right_stick_x;
    theta = -_7ByawPitchRollAnglesVariable_7D.getYaw(AngleUnit.DEGREES);
    // Calculated Values
    rotX = x * Math.cos(theta / 180 * Math.PI) - y * Math.sin(theta / 180 * Math.PI);
    rotY = x * Math.sin(theta / 180 * Math.PI) + y * Math.cos(theta / 180 * Math.PI);
  }

  /**
   * Describe this function...
   */
  private void Slides(int SlidesTicks, int SlidesVelocity) {
    SlideL.setTargetPosition(SlidesTicks);
    SlideR.setTargetPosition(SlidesTicks);
    SlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    SlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    ((DcMotorEx) SlideL).setVelocity(SlidesVelocity);
    ((DcMotorEx) SlideR).setVelocity(SlidesVelocity);
  }

  /**
   * Describe this function...
   */
  private void setPower() {
    // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
    // We negate this value so that the topmost position corresponds to maximum forward power.
    leftFront.setPower((rotY + rotX + rx) * multiplier);
    rightFront.setPower(((rotY - rotX) - rx) * multiplier);
    // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
    // We negate this value so that the topmost position corresponds to maximum forward power.
    leftBack.setPower(((rotY - rotX) + rx) * multiplier);
    rightBack.setPower(((rotY + rotX) - rx) * multiplier);
  }

  /**
   * Describe this function...
   */
  private void GripperArm(int Ticks2, double Power2) {
    gripperArm.setTargetPosition(Ticks2);
    gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    gripperArm.setPower(Power2);
  }

  /**
   * Describe this function...
   */
  private void data() {
    telemetry.addData("IMU Yaw:", _7ByawPitchRollAnglesVariable_7D.getYaw(AngleUnit.DEGREES));
    telemetry.addData("rotX", rotX);
    telemetry.addData("rotY", rotY);
    telemetry.addData("Y", y);
    telemetry.addData("X", x);
    telemetry.addData("Theta (Radians)", _7ByawPitchRollAnglesVariable_7D.getYaw(AngleUnit.RADIANS));
    telemetry.addData("rx", rx);
    telemetry.addData("Multiplier (Speed)", multiplier);
    telemetry.addData("armBase Ticks", armBase.getCurrentPosition());
    telemetry.addData("gripperArm Ticks", gripperArm.getCurrentPosition());
    telemetry.addData("gripperArm Current", ((DcMotorEx) gripperArm).getCurrent(CurrentUnit.AMPS));
    telemetry.addData("armBase Current", ((DcMotorEx) armBase).getCurrent(CurrentUnit.AMPS));
    telemetry.addData("SlideL Current", ((DcMotorEx) SlideL).getCurrent(CurrentUnit.AMPS));
    telemetry.addData("SlideR Current", ((DcMotorEx) SlideR).getCurrent(CurrentUnit.AMPS));
    telemetry.update();
  }

  /**
   * Describe this function...
   */
  private void ArmBase(int Ticks, double Power) {
    armBase.setTargetPosition(Ticks);
    armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armBase.setPower(Power);
  }
}
