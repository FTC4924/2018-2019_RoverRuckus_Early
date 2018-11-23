
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "linearServoTeleop", group = "Iterative Opmode")
public class linearServoTeleop extends OpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private Servo linearServo = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        final double MIDDLEPOSITION180 = 0.0;
        //final double MIDDLE_POSITION_CONTINOUS = 0.0;
        telemetry.addData("Status", "Initialized");

        linearServo = hardwareMap.get(Servo.class, "linearServo");

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
        

        if (gamepad2.left_bumper) {
            linearServo.setPosition(0);

        } else if (gamepad2.right_bumper) {
            linearServo.setPosition(1);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}