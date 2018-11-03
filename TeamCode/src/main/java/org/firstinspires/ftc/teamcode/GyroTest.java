package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;



import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name="GyroTest", group="Pushbot")

public class GyroTest extends LinearOpMode {


    ColorSensor sensorColorRight;
    ColorSensor sensorColorLeft;
    ColorSensor sensorColorMiddle;
    // DistanceSensor sensorDistanceL;
    DistanceSensor sensorDistanceR;
    TouchSensor limitSwitch;
    Servo rightArm;
    // Servo leftArm;

    //Test variable used for current model
    //Will be deleted soon
    final int WHITE_ALPHA = 150;

    //Booleans used for while loops
    boolean kicked = false;
    boolean notLowered = false;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor collection = null;
    private DcMotor linearMotor = null;
    private Servo linearServo = null;
    private CRServo collectionServo = null;
    double clawClosePosition = 0.5;

    //Variables used for encoder calculations
    static final double     COUNTS_PER_MOTOR_REV    = 1425.2 ;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.25;

    static BNO055IMU imu;

    static BNO055IMU.Parameters IMU_Parameters = new BNO055IMU.Parameters();
    Orientation angles;

    protected static DcMotor[] DRIVE_BASE_MOTORS = new DcMotor[4];

    private static final double GYRO_TURN_TOLERANCE_DEGREES = 5;




    @Override
    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");
        collection = hardwareMap.get(DcMotor.class, "collection");
        linearMotor = hardwareMap.get(DcMotor.class, "linearMotor");
        collectionServo = hardwareMap.get(CRServo.class, "collectionServo");
        linearServo = hardwareMap.get(Servo.class, "linearServo");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        collection.setDirection(DcMotor.Direction.FORWARD);
        linearMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        DRIVE_BASE_MOTORS[0] = frontLeftMotor;
        DRIVE_BASE_MOTORS[1] = frontRightMotor;
        DRIVE_BASE_MOTORS[2] = backLeftMotor;
        DRIVE_BASE_MOTORS[3] = backRightMotor;

        telemetry.addData("Status", "Initialized");    //
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            collectionServo.setPower(1);
            sleep(200);
            collectionServo.setPower(0);
            turnToPosition(45, 90);

        }
    }

    public void turn(double power) {
            frontLeftMotor.setPower(power);
            frontRightMotor.setPower(-power);
            backLeftMotor.setPower(power);
            backRightMotor.setPower(-power);
    }

    public void turnToPosition(double turnPower, double desiredHeading) {

        setMotorsModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER, DRIVE_BASE_MOTORS);

        while (getHeading() - desiredHeading > GYRO_TURN_TOLERANCE_DEGREES) {

            telemetry.addData("Heading", getHeading());
            telemetry.update();
            turn(turnPower);
        }
        while (getHeading() - desiredHeading < -GYRO_TURN_TOLERANCE_DEGREES) {

            telemetry.addData("Heading", getHeading());
            telemetry.update();
            turn(turnPower);
        }
        setMotorsPowers(0, DRIVE_BASE_MOTORS);
        //defaults to CW turning
    }

    public double getHeading() {
        updateGyro();
        return angles.firstAngle;
    }

    void updateGyro() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public static void setMotorsModes(DcMotor.RunMode runMode, DcMotor[] motors) {

        for (DcMotor d : motors)
            d.setMode(runMode);
    }

    protected static void setMotorsPowers(double power, DcMotor[] motors) {

        for (DcMotor d : motors)
            d.setPower(power);
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = frontLeftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = frontRightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            frontLeftMotor.setTargetPosition(newLeftTarget);
            backLeftMotor.setTargetPosition(newLeftTarget);
            frontRightMotor.setTargetPosition(newRightTarget);
            backRightMotor.setTargetPosition(newRightTarget);


            // Turn On RUN_TO_POSITION
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontRightMotor.setPower(Math.abs(speed));
            frontLeftMotor.setPower(Math.abs(speed));
            backRightMotor.setPower(Math.abs(speed));
            backLeftMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftMotor.isBusy() && frontRightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to " + newLeftTarget + newRightTarget);
                telemetry.addData("Path2",  "Running at " +
                        frontLeftMotor.getCurrentPosition() +
                        frontRightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}