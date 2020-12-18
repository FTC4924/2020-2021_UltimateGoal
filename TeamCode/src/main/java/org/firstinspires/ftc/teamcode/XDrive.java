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

    private boolean bPressed;
    private boolean xPressed;
    private boolean bristlesIn;
    private boolean bristlesOut;

    private boolean yPressed;
    private boolean shooterRev;

    private boolean rightBumperPressed;
    private boolean leftBumperPressed;
    private int elevatorPositionIndex;

    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    public DcMotor bristles;
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

        bPressed = false;
        xPressed = false;
        bristlesIn = false;
        bristlesOut = false;
        yPressed = false;
        shooterRev = false;
        rightBumperPressed = false;
        leftBumperPressed = false;

        elevatorPositionIndex = 0;

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        bristles = hardwareMap.get(DcMotor.class, "collection");
        elevator = hardwareMap.get(Servo.class, "elevator");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        funnelLeft = hardwareMap.get(Servo.class, "funnelLeft");
        funnelRight = hardwareMap.get(Servo.class, "funnelRight");

    }

    public void loop() {

        gamepad1LeftStickX = gamepad1.left_stick_x;
        gamepad1LeftStickY = gamepad1.left_stick_y;
        gamepad1RightStickX = gamepad1.right_stick_x;
        gamepad1RightStickY = gamepad1.right_stick_y;

        /* if the joystick is far enough from the center the robot will move in the direction of the
        joystick based on the robot's perspective. */
        if (Math.abs(gamepad1LeftStickX) > JOYSTICK_TOLERANCE || Math.abs(gamepad1LeftStickY) > JOYSTICK_TOLERANCE) {
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

        if (Math.abs(gamepad1RightStickX) > JOYSTICK_TOLERANCE) {
            leftFrontPower -= gamepad1RightStickX / TURNING_REDUCTION;
            leftBackPower -= gamepad1RightStickX / TURNING_REDUCTION;
            rightBackPower -= gamepad1RightStickX / TURNING_REDUCTION;
            rightFrontPower -= gamepad1RightStickX / TURNING_REDUCTION;
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

        if (gamepad2.right_bumper) {
            if(!rightBumperPressed) {
                rightBumperPressed = true;
                elevatorPositionIndex = (elevatorPositionIndex + 1) % 5;
            }
        } else {
            rightBumperPressed = false;
        }
        if (gamepad2.left_bumper) {
            if(!leftBumperPressed) {
                leftBumperPressed = true;
                if(elevatorPositionIndex > 0) {
                    elevatorPositionIndex = (elevatorPositionIndex - 1) % 5;
                }
            }
        } else {
            leftBumperPressed = false;
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

        switch (elevatorPositionIndex) {

            case 0:
                elevator.setPosition(ElevatorPositions.DOWN.positionValue);
            case 1:
                elevator.setPosition(ElevatorPositions.MIDDLE.positionValue);
            case 2:
                elevator.setPosition(ElevatorPositions.RING_ONE.positionValue);
            case 3:
                elevator.setPosition(ElevatorPositions.RING_TWO.positionValue);
            case 4:
                elevator.setPosition(ElevatorPositions.RING_THREE.positionValue);

        }

        if (shooterRev) {
            shooter.setPower(1.0);
        } else {
            shooter.setPower(0.0);
        }
    }
}
