package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="ArnavTest", group="Linear Opmode")
public class ArnavTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    TouchSensor limitSwitch;
    Servo linearServo;
    DcMotor motor;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor frontLeft;

    final int WHITE_ALPHA = 150;

    boolean kicked = false;

    @Override
    public void runOpMode() {

        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
        linearServo = hardwareMap.get(Servo.class, "linearServo");
        motor = hardwareMap.get(DcMotor.class, "linearMotor");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (!kicked){

                linearServo.scaleRange(0.0, 1.0);

                frontRight.setDirection(DcMotor.Direction.REVERSE);
                backRight.setDirection(DcMotor.Direction.REVERSE);
                motor.setDirection(DcMotor.Direction.REVERSE);

                motor.setPower(1);

                    if (limitSwitch.isPressed()) {
                        kicked = true;
                        motor.setPower(0);
                        linearServo.setPosition(1);
                        sleep(5000);
                        frontRight.setPower(.5);
                        frontLeft.setPower(.5);
                        backLeft.setPower(.5);
                        backRight.setPower(.5);
                    }

            }

        }
    }
}
