package org.firstinspires.ftc.teamcode.Old;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Encoder", group="Pushbot")
@Disabled
public class Encoders extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    ColorSensor sensorColorRight;
    ColorSensor sensorColorLeft;
    ColorSensor sensorColorMiddle;
   // DistanceSensor sensorDistanceL;
    DistanceSensor sensorDistanceR;
    TouchSensor limitSwitch;
    Servo rightArm;
   // Servo leftArm;
    Servo linearServo;

    //Test variable used for current model
    //Will be deleted soon
    final int WHITE_ALPHA = 150;

    //Booleans used for while loops
    boolean kicked = false;
    boolean notLowered = false;
    boolean lowered = true;


    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor linearMotor;

    //Variables used for encoder calculations
    static final double     COUNTS_PER_MOTOR_REV    = 1425.2 ;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.25;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        linearMotor = hardwareMap.get(DcMotor.class,  "linearMotor");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        linearMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sensorColorRight = hardwareMap.get(ColorSensor.class, "rightColor");
        sensorColorLeft = hardwareMap.get(ColorSensor.class, "leftColor");
        sensorColorMiddle = hardwareMap.get(ColorSensor.class, "middleColor");
        //sensorDistanceL = hardwareMap.get(DistanceSensor.class, "leftColor");
        sensorDistanceR = hardwareMap.get(DistanceSensor.class, "rightColor");
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
        //leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        linearServo = hardwareMap.get(Servo.class, "linearServo");

        //Set the range for the linear servo
        linearServo.scaleRange(0.0, 1.0);

        telemetry.addData("Status", "Initialized");    //
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            //Turns on motor to power lead screw in order to go down
            linearMotor.setPower(1);

            //Checks if we have reached the limit of the lead screw, which activates the limit switch
            //This will insure that we are down on the ground
            while (notLowered) {
                if (limitSwitch.isPressed()) {
                    kicked = true;
                    linearMotor.setPower(0);
                    linearServo.setPosition(1);
                    sleep(200);
                }
            }
         while (lowered && opModeIsActive()) {
             //Drives forward 5 inches
             encoderDrive(DRIVE_SPEED, 5, 5, 5.0);

             //Positions the arms so that the color sensor is right in front of the minerals
             rightArm.setPosition(0.8);
             // leftArm.setPosition(0.2);

             //Insures that the reading is when the arms are on the ground by waiting 2 seconds
             sleep(2000);

             if (!kicked) {

                 //Publishes data on the values that we are reading from both the color and distance sensors
                 telemetry.addData("Alpha Left", sensorColorLeft.alpha());
                 telemetry.addData("Alpha Middle", sensorColorMiddle.alpha());
                 telemetry.addData("Alpha Right", sensorColorRight.alpha());
                 // telemetry.addData("Distance Left", sensorDistanceL.getDistance(DistanceUnit.CM));
                 telemetry.addData("Distance Right", sensorDistanceR.getDistance(DistanceUnit.CM));

                 //Puts all of the published data into a log, in order to read later on
                 Log.i("Alpha Left", "" + sensorColorLeft.alpha());
                 Log.i("Alpha Middle", "" + sensorColorMiddle.alpha());
                 Log.i("Alpha Right", "" + sensorColorRight.alpha());
                 // Log.i("Distance Left", "" + sensorDistanceL.getDistance(DistanceUnit.CM));
                 Log.i("Distance Right", "" + sensorDistanceR.getDistance(DistanceUnit.CM));

                 //Determines what way to kick, in order to kick the yellow side
                 if (sensorColorLeft.alpha() < WHITE_ALPHA) {
                     telemetry.addData("Action", "Right Up");
                     rightArm.setPosition(0.2);
                     kicked = true;
                 } else if (sensorColorRight.alpha() < WHITE_ALPHA) {
                     telemetry.addData("Action", "Left Up");
                     // leftArm.setPosition(0.8);
                     kicked = true;
                 } else if (sensorColorMiddle.alpha() < WHITE_ALPHA) {
                     telemetry.addData("Action", "Go Forward");
                     rightArm.setPosition(0);
                     // leftArm.setPosition(1);
                     kicked = true;
                 } else {
                     telemetry.addData("Action", "Confused");
                 }

                 telemetry.update();

                 sleep(5000);
             }
             public void encoderDrive ( double speed,
             double leftInches, double rightInches,
             double timeoutS){
                 int newLeftTarget;
                 int newRightTarget;

                 // Ensure that the opmode is still active
                 if (opModeIsActive()) {

                     // Determine new target position, and pass to motor controller
                     newLeftTarget = frontLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
                     newRightTarget = frontRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
                     frontLeft.setTargetPosition(newLeftTarget);
                     backLeft.setTargetPosition(newLeftTarget);
                     frontRight.setTargetPosition(newRightTarget);
                     backRight.setTargetPosition(newRightTarget);


                     // Turn On RUN_TO_POSITION
                     frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                     backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                     frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                     backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                     // reset the timeout time and start motion.
                     runtime.reset();
                     frontRight.setPower(Math.abs(speed));
                     frontLeft.setPower(Math.abs(speed));
                     backRight.setPower(Math.abs(speed));
                     backLeft.setPower(Math.abs(speed));

                     while (opModeIsActive() &&
                             (runtime.seconds() < timeoutS) &&
                             (frontLeft.isBusy() && frontRight.isBusy())) {

                         // Display it for the driver.
                         telemetry.addData("Path1", "Running to " + newLeftTarget + newRightTarget);
                         telemetry.addData("Path2", "Running at " +
                                 frontLeft.getCurrentPosition() +
                                 frontRight.getCurrentPosition());
                         telemetry.update();
                     }

                     // Stop all motion;
                     frontLeft.setPower(0);
                     frontRight.setPower(0);
                     backLeft.setPower(0);
                     backRight.setPower(0);

                     // Turn off RUN_TO_POSITION
                     frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                     frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                     backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                     backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                 }
             }

         }
        }

    }

