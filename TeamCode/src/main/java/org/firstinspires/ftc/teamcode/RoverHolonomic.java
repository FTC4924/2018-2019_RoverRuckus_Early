
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "RoverHolonomic", group = "Iterative Opmode")

public class RoverHolonomic extends OpMode {
    // Declare OpMode members.
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
    /*

     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        final double MIDDLEPOSITION180 = 0.0;
        //final double MIDDLE_POSITION_CONTINOUS = 0.0;
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
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

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        linearServo.scaleRange(0.0, 1.0);

        double collectionPower = 0.0;
        double sideways = 0.0;
        double extendCollection = 0.0;
        boolean extended = true;
        double deliveryPower = 0.0;
        boolean barDownPosition;
        final double BARMOVE = 1.0;
        final double BARCLOSE = 0.0;
        boolean elbowBent;
        double position = 0.0;
        double clawPosition = 0.0;

        //we set what to do when the motor is not given power, which is to brake completely, instead of coasting
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        double drive = -gamepad1.left_stick_y;
        //drive is what direction we want to move, either forwards, backwards, or neither
        double holonomic = -gamepad1.left_stick_x;
        //holonomic is what direction we want to move sideways
        double turnRight = gamepad1.right_trigger;
        //turnRight is how much we want to turn right
        double turnLeft = gamepad1.left_trigger;
        //turnLeft is how much we want to turn left
        boolean collectionPowerUp = gamepad2.b;
        //collectionPowerUp is dependent on whether or not we want the collection to collect
        boolean collectionPowerDown = gamepad2.a;
        //collectionPowerDown is dependent on whether or not we want the collection deliver (Push downwards)

        double leadScrew =  (gamepad2.left_stick_y);

        boolean hookMovementLeft = gamepad2.dpad_left;
        boolean hookMovementRight = gamepad2.dpad_right;

        boolean collectionUp = gamepad2.y;
        boolean collectionDown = gamepad2.x;

        boolean halfSpeed = gamepad1.left_bumper;

        if (collectionPowerUp) {
            //if we want it to collect, we set collectionPower to 1
            collectionPower = 1;
        } else if (collectionPowerDown) {
            //if we want the collection to deliver/spin backswards, we set collectionPower to -1
            collectionPower = -1;
        }

        if (hookMovementLeft) {
            //if we want it to collect, we set collectionPower to 1
            linearServo.setPosition(0);

        } else if (hookMovementRight) {
            //if we want the collection to deliver/spin backswards, we set collectionPower to -1
            linearServo.setPosition(0.9);
        }

        if (collectionUp ) {
            //if we want it to collect, we set collectionPower to 1
            extendCollection = 1;
        } else if (collectionDown) {
            //if we want the collection to deliver/spin backswards, we set collectionPower to -1
            extendCollection = -1;
        }

       /* if (gamepad2.left_bumper) {
            barServo.setPosition(BARCLOSE + BARMOVE);
            clawPosition = BARCLOSE + BARMOVE;
        } else  {
            barServo.setPosition(BARCLOSE);
            clawPosition = BARCLOSE;
        }

        if (gamepad2.right_bumper) {
            elbowServo.setPower(gamepad2.right_trigger);
        } else {
            elbowServo.setPower(-gamepad2.left_trigger);
        }

        if (deliveryUp) {
            deliveryPower = -1;
        } else if (deliveryDown) {
            //if we want the collection to deliver/spin backswards, we set collectionPower to -1
            deliveryPower = 1;
        } */

        //we are calculating the power to send to each different wheel, which each need their own power since it is calculated in different ways

        double frontLeftPower =  Range.clip(drive - holonomic + turnRight - turnLeft, -1.0, 1.0);
        double frontRightPower =  Range.clip(drive + holonomic - turnRight + turnLeft, -1.0, 1.0);
        double backRightPower =  Range.clip(drive - holonomic - turnRight + turnLeft, -1.0, 1.0);
        double backLeftPower =  Range.clip(drive + holonomic + turnRight - turnLeft, -1.0, 1.0);

        if (halfSpeed) {
            frontLeftPower = 0.5 * (frontLeftPower);
            frontRightPower = 0.5 * (frontRightPower);
            backRightPower = 0.5 * (backRightPower);
            backLeftPower = 0.5 * (backLeftPower);

        }

        // Send calculated power to wheels and motors
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
        collection.setPower(collectionPower);
        linearMotor.setPower(leadScrew);
        collectionServo.setPower(extendCollection);

        // Show the elapsed game time
        // telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Slow Mode", halfSpeed);
        telemetry.addData("Elbow Servo", "Continous Position" + position);
        telemetry.addData("Claw Servo", "Continous Position" + clawPosition);

        turnLeft = 0;
        turnRight = 0;

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}