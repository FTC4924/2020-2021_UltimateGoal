package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
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

    private double currentRobotAngle;

    private boolean bPressed;
    private boolean xPressed;
    private boolean bristlesIn;
    private boolean bristlesOut;

    private boolean yPressed;
    private boolean shooterRev;

    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    public DcMotor bristles;
    public Servo elevator;
    public DcMotor shooter;
    public Servo funnelLeft;
    public Servo funnelRight;

    BNO055IMU imu;
    Orientation angles;
    BNO055IMU.Parameters parameters;

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

        currentRobotAngle = 0.0;

        bPressed = false;
        xPressed = false;
        bristlesIn = false;
        bristlesOut = false;

        yPressed = false;
        shooterRev = false;

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        bristles = hardwareMap.get(DcMotor.class, "collection");
        elevator = hardwareMap.get(Servo.class, "elevator");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        funnelLeft = hardwareMap.get(Servo.class, "funnelLeft");
        funnelRight = hardwareMap.get(Servo.class, "funnelRight");

        parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        angles = null;

    }

    public void loop() {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);;

        currentRobotAngle = angles.firstAngle;

        gamepad1LeftStickX = gamepad1.left_stick_x;
        gamepad1LeftStickY = gamepad1.left_stick_y;
        gamepad1RightStickX = gamepad1.right_stick_x;
        gamepad1RightStickY = gamepad1.right_stick_y;
        gamepad1LeftStickAngle = Math.atan2(gamepad1LeftStickY, gamepad1LeftStickX);

        if(Math.abs(gamepad1LeftStickX) >= JOYSTICK_TOLERANCE || Math.abs(gamepad1LeftStickY) >= JOYSTICK_TOLERANCE) {
            leftFrontPower = Math.cos((gamepad1LeftStickAngle + (Math.PI / 4)) + Math.toRadians(currentRobotAngle)) * -1;
            leftBackPower = Math.sin((gamepad1LeftStickAngle + (Math.PI / 4)) + Math.toRadians(currentRobotAngle));
            rightFrontPower = Math.sin((gamepad1LeftStickAngle + (Math.PI / 4)) + Math.toRadians(currentRobotAngle)) * -1;
            rightBackPower = Math.cos((gamepad1LeftStickAngle + (Math.PI / 4)) + Math.toRadians(currentRobotAngle));

            leftFrontPower *= ((-1 * gamepad1RightStickY) + 1) / 2;
            leftBackPower *= ((-1 * gamepad1RightStickY) + 1) / 2;
            rightFrontPower *= ((-1 * gamepad1RightStickY) + 1) / 2;
            rightBackPower *= ((-1 * gamepad1RightStickY) + 1) / 2;
        } else {
            leftFrontPower = 0.0;
            leftBackPower = 0.0;
            rightFrontPower = 0.0;
            rightBackPower = 0.0;
        }

        if (Math.abs(gamepad1RightStickX) > JOYSTICK_TOLERANCE) {
            leftFrontPower -= gamepad1RightStickX / TURNING_REDUCTION;
            leftBackPower -= gamepad1RightStickX / TURNING_REDUCTION;
            rightFrontPower -= gamepad1RightStickX / TURNING_REDUCTION;
            rightBackPower -= gamepad1RightStickX / TURNING_REDUCTION;
        }

        if (gamepad2.b) {
            if (!bPressed) {
                bPressed = true;
                bristlesIn = !bristlesIn;
                if (bristlesIn) {
                    bristlesOut = false;
                }
            }
        } else {
            bPressed = false;
        }
        if (gamepad2.x) {
            if (!xPressed) {
                xPressed = true;
                bristlesOut = !bristlesOut;
                if (bristlesOut) {
                    bristlesIn = false;
                }
            }
        } else {
            xPressed = false;
        }

        if (gamepad2.y) {
            if (!yPressed) {
                yPressed = true;
                shooterRev = !shooterRev;
            }
        } else {
            yPressed = false;
        }

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);

        if (bristlesIn) {
            bristles.setPower(-0.75);
        } else if (bristlesOut) {
            bristles.setPower(0.75);
        } else {
            bristles.setPower(0.0);
        }

        if (shooterRev) {
            shooter.setPower(-1.0);
        } else {
            shooter.setPower(0.0);
        }
    }
}