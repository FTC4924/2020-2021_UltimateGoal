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
import static org.firstinspires.ftc.teamcode.Constants.*;

@TeleOp(name="AdvancedXDrive")
public class AdvancedXDrive extends OpMode {
    //Creating variables so the robot works
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

    private boolean rightBumperPressed;
    private boolean leftBumperPressed;
    private int elevatorPositionIndex;

    //creating the variables for the gyro sensor
    BNO055IMU imu;
    Orientation angles;
    BNO055IMU.Parameters parameters;

    public void init() {
        //Setting initial values for the variables so the robot has a place to start
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
        //Mapping the variables to their appropriate motor/servo so the robot understands the code
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
        //initializing the gyro sensor
        parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        angles = null;
        elevatorPositionIndex = 0;

    }

    public void loop() {
        telemetry.addData("Elevator Index", elevatorPositionIndex);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);;

        currentRobotAngle = angles.firstAngle;
        //Initializing the controller joysticks
        gamepad1LeftStickX = gamepad1.left_stick_x;
        gamepad1LeftStickY = gamepad1.left_stick_y;
        gamepad1RightStickX = gamepad1.right_stick_x;
        gamepad1RightStickY = gamepad1.right_stick_y;
        gamepad1LeftStickAngle = Math.atan2(gamepad1LeftStickY, gamepad1LeftStickX);

        //This makes the robot move according to the 'front' of the field based on its position as we start the program
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
        //Turning the robot
        if (gamepad1.left_trigger >= JOYSTICK_TOLERANCE) {
            leftFrontPower += Math.pow(gamepad1.left_trigger, 2);
            leftBackPower += Math.pow(gamepad1.left_trigger, 2);
            rightFrontPower += Math.pow(gamepad1.left_trigger, 2);
            rightBackPower += Math.pow(gamepad1.left_trigger, 2);
        }

        if (gamepad1.right_trigger >= JOYSTICK_TOLERANCE) {
            leftFrontPower -= Math.pow(gamepad1.right_trigger, 2);
            leftBackPower -= Math.pow(gamepad1.right_trigger, 2);
            rightFrontPower -= Math.pow(gamepad1.right_trigger, 2);
            rightBackPower -= Math.pow(gamepad1.right_trigger, 2);
        }
        //Pressing the B button makes bristles spin out. Press it again to stop the bristles
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
        //Pressing the X button spins bristles in. Press it again to stop the bristles.
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
        //Cycling through the elevator positions
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
        //Pressing the Y button makes the shooter spin
        if (gamepad2.y) {
            if (!yPressed) {
                yPressed = true;
                shooterRev = !shooterRev;
            }
        } else {
            yPressed = false;
        }
        //Setting the power for the drive train based on joystick values
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
        //Setting the power for the rotation of the bristles
        if (bristlesIn) {
            bristles.setPower(-0.75);
        } else if (bristlesOut) {
            bristles.setPower(0.75);
        } else {
            bristles.setPower(0.0);
        }
        //Setting the elevator to the position depending on the elevator position index
        switch (elevatorPositionIndex) {

            case 0:
                elevator.setPosition(ElevatorPositions.DOWN.positionValue);
                break;
            case 1:
                elevator.setPosition(ElevatorPositions.MIDDLE.positionValue);
                break;
            case 2:
                elevator.setPosition(ElevatorPositions.RING_ONE.positionValue);
                break;
            case 3:
                elevator.setPosition(ElevatorPositions.RING_TWO.positionValue);
                break;
            case 4:
                elevator.setPosition(ElevatorPositions.RING_THREE.positionValue);

        }
        //Turning the shooter on and off
        if (shooterRev) {
            shooter.setPower(-1.0);
        } else {
            shooter.setPower(0.0);
        }
    }
}