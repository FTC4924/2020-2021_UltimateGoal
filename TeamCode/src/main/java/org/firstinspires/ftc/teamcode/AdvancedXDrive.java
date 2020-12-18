package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Constants.JOYSTICK_TOLERANCE;
import static org.firstinspires.ftc.teamcode.Constants.TURNING_REDUCTION;

@TeleOp(name="AdvancedXDrive")
public class AdvancedXDrive extends OpMode {

    private double gamepad1LeftStickX;
    private double gamepad1LeftStickY;
    private double gamepad1RightStickX;
    private double gamepad1RightStickY;

    private double gamepad1LeftStickAngle;

    private double leftFrontPower;
    private double leftBackPower;
    private double rightFrontPower;
    private double rightBackPower;

    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    public void init() {

        gamepad1LeftStickX = 0.0;
        gamepad1LeftStickY = 0.0;
        gamepad1RightStickX = 0.0;
        gamepad1RightStickY = 0.0;

        gamepad1LeftStickAngle = 0.0;

        leftFrontPower = 0.0;
        leftBackPower = 0.0;
        rightFrontPower = 0.0;
        rightBackPower = 0.0;

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

    }
    public void loop() {

        gamepad1LeftStickX = gamepad1.left_stick_x;
        gamepad1LeftStickY = gamepad1.left_stick_y;
        gamepad1RightStickX = gamepad1.right_stick_x;
        gamepad1RightStickY = gamepad1.right_stick_y;

        gamepad1LeftStickAngle = Math.atan2(gamepad1LeftStickY, gamepad1LeftStickX);
        leftFrontPower = Math.cos(gamepad1LeftStickAngle-(Math.PI/4));
        leftBackPower = Math.sin(gamepad1LeftStickAngle-(Math.PI/4));
        rightFrontPower = Math.sin(gamepad1LeftStickAngle-(Math.PI/4))*-1;
        rightBackPower = Math.cos(gamepad1LeftStickAngle-(Math.PI/4))*-1;

        leftFrontPower *= (gamepad1RightStickY + 1) / 2;
        leftBackPower *= (gamepad1RightStickY + 1) / 2;
        rightBackPower *= (gamepad1RightStickY + 1) / 2;
        rightFrontPower *= (gamepad1RightStickY + 1) / 2;

        if (Math.abs(gamepad1RightStickX) > JOYSTICK_TOLERANCE) {
            leftFrontPower -= gamepad1RightStickX / TURNING_REDUCTION;
            leftBackPower -= gamepad1RightStickX / TURNING_REDUCTION;
            rightBackPower -= gamepad1RightStickX / TURNING_REDUCTION;
            rightFrontPower -= gamepad1RightStickX / TURNING_REDUCTION;
        }


    }
}