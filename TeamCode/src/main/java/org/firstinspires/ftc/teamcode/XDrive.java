package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Constants.*;

@TeleOp(name="XDrive")
public class XDrive extends OpMode {

    private double gamepad1LeftStickY;
    private double gamepad1LeftStickX;
    private double gamepad1RightStickX;
    private double gamepad1RightStickY;

    private double leftFrontPower;
    private double leftBackPower;
    private double rightFrontPower;
    private double rightBackPower;

    private boolean bristlesIn;
    private boolean bristlesOut;
    private boolean bPressed;
    private boolean xPressed;

    private boolean yPressed;
    private boolean shooterRev;

    private int elevatorPosition;

    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    public DcMotor collection;
    public Servo elevator;
    public DcMotor shooter;

    public Servo funnelLeft;
    public Servo funnelRight;

    public void init() {

        gamepad1LeftStickY = 0.0;
        gamepad1LeftStickX = 0.0;
        gamepad1RightStickX = 0.0;
        gamepad1RightStickY = 0.0;

        leftFrontPower = 0.0;
        leftBackPower = 0.0;
        rightFrontPower = 0.0;
        rightBackPower = 0.0;

        elevatorPosition = 0;

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        collection = hardwareMap.get(DcMotor.class, "collection");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        elevator = hardwareMap.get(Servo.class, "elevator");

        funnelLeft = hardwareMap.get(Servo.class, "funnelLeft");
        funnelRight = hardwareMap.get(Servo.class, "funnelRight");

    }
    public void loop() {
        gamepad1LeftStickX = gamepad1.left_stick_x;
        gamepad1LeftStickY = gamepad1.left_stick_y;
        gamepad1RightStickX = gamepad1.right_stick_x;
        gamepad1RightStickY = gamepad1.right_stick_y;

        /* if the joystick is far enough from the center the robot will move in the direction the
        joystick based on the robot's perspective. */
        if (Math.abs(gamepad1LeftStickX) > 0.05 || Math.abs(gamepad1LeftStickY) > 0.05) {

            leftFrontPower = -gamepad1LeftStickX + gamepad1LeftStickY;
            rightFrontPower = -gamepad1LeftStickX - gamepad1LeftStickY;
            leftBackPower = gamepad1LeftStickX + gamepad1LeftStickY;
            rightBackPower = gamepad1LeftStickX - gamepad1LeftStickY;

        } else {

            leftFrontPower = 0.0;
            leftBackPower = 0.0;
            rightFrontPower = 0.0;
            rightBackPower = 0.0;

        }

        if (Math.abs(gamepad1RightStickX) > 0.05) {

            leftFrontPower -= gamepad1RightStickX / 2.5;
            leftBackPower -= gamepad1RightStickX / 2.5;
            rightBackPower -= gamepad1RightStickX / 2.5;
            rightFrontPower -= gamepad1RightStickX / 2.5;

        }

        if (gamepad2.b) {
            if (!bPressed) {
                bPressed = true;
                if (bristlesIn) {
                    bristlesIn = false;
                } else {
                    bristlesIn = true;
                    bristlesOut = false;
                }
            }
        } else {
            bPressed = false;
        }
        if (gamepad2.x) {
            if (!xPressed) {
                xPressed = true;
                if (bristlesOut) {
                    bristlesOut = false;
                } else {
                    bristlesOut = true;
                    bristlesIn = false;
                }
            } 
        } else {
            xPressed = false;
        }
        if (gamepad2.y) {
            if (!yPressed) {
                yPressed = true;
                if (shooterRev) {
                    shooterRev = false;
                } else {
                    shooterRev = true;
                }
            }
        } else {
            yPressed = false;
        }

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);

        if (bristlesIn) {
            collection.setPower(-0.75);
        } else if (bristlesOut) {
            collection.setPower(0.75);
        } else if (!bristlesIn && !bristlesOut) {
            collection.setPower(0);
        }
    }
}
