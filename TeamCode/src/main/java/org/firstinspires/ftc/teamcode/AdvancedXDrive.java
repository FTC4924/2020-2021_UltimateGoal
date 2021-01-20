package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.teamcode.Constants.*;

@TeleOp(name="AdvancedXDrive")
public class AdvancedXDrive extends OpMode {

    private boolean bPressed;
    private boolean xPressed;
    private boolean rightStickPressed;
    private boolean collectionMaxPower;
    private boolean bristlesIn;
    private boolean bristlesOut;

    private boolean rightBumperPressed;
    private boolean leftBumperPressed;
    private byte elevatorPositionIndex;

    private double shooterLifterTargetPosition;

    private boolean yPressed;
    private boolean shooterRev;

    private boolean aPressed;
    private boolean funnelDown;

    //Motors and servos
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor bristles;
    private Servo elevator;
    private Servo kicker;
    private DcMotor shooter;
    private Servo funnelLeft;
    private Servo funnelRight;
    private Servo shooterLifterLeft;
    private Servo shooterLifterRight;
    private Servo conveyor;


    //Creating the variables for the gyro sensor
    private BNO055IMU imu;

    private Orientation angles;
    private double angleOffset;
    private double currentRobotAngle;

    public void init() {

        angleOffset = 0.0;
        currentRobotAngle = 0.0;

        bPressed = false;
        xPressed = false;
        bristlesIn = false;
        bristlesOut = false;

        rightBumperPressed = false;
        leftBumperPressed = false;
        elevatorPositionIndex = 0;

        shooterLifterTargetPosition = SHOOTER_LIFTER_DEFAULT_POSITION;

        yPressed = false;
        shooterRev = false;

        /*Instantiating the motor and servo objects as their appropriate motor/servo in the
        configuration on the robot*/
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        bristles = hardwareMap.get(DcMotor.class, "collection");
        //conveyor = hardwareMap.get(Servo.class, "conveyor"); Not needed for first competition!
        elevator = hardwareMap.get(Servo.class, "elevator");
        kicker = hardwareMap.get(Servo.class, "kicker");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooterLifterLeft = hardwareMap.get(Servo.class, "shooterLeft");
        shooterLifterRight = hardwareMap.get(Servo.class, "shooterRight");
        funnelLeft = hardwareMap.get(Servo.class, "funnelLeft");
        funnelRight = hardwareMap.get(Servo.class, "funnelRight");

        //Initializing the revhub IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = null;
        currentRobotAngle = 0.0;

    }

    public void loop() {

        telemetry.addData("angleOffset", angleOffset);

        //getting the angle of the robot from the IMU
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, RADIANS);
        currentRobotAngle = angles.firstAngle - angleOffset;

        recalibrateGyro();

        holonomicDrive();

        funnel();
        bristles();

        elevator();

        kicker();

        shooterLifter();
        shooterWheel();

    }

    private void recalibrateGyro() {

        if(gamepad1.b) {
            angleOffset = angles.firstAngle;
        }

    }

    private void holonomicDrive() {

        double gamepad1LeftStickX = gamepad1.left_stick_x;
        double gamepad1LeftStickY = gamepad1.left_stick_y;
        double gamepad1RightStickY = gamepad1.right_stick_y;
        double gamepad1LeftTrigger = gamepad1.left_trigger;
        double gamepad1RightTrigger = gamepad1.right_trigger;

        double leftFrontPower;
        double leftBackPower ;
        double rightFrontPower;
        double rightBackPower;

        /*Holonomic controls according to what direction the robot is facing when we start the
        program*/
        if (Math.abs(gamepad1LeftStickX) >= TOLERANCE || Math.abs(gamepad1LeftStickY) >= TOLERANCE) {
            //Uses atan2 to convert the x and y values of the controller to an angle
            double gamepad1LeftStickAngle = Math.atan2(gamepad1LeftStickY, gamepad1LeftStickX);

            /*Determines what power each wheel should get based on the angle we get from the stick
            plus the current robot angle so that the controls are independent of what direction the
            robot is facing*/
            leftFrontPower = Math.cos(gamepad1LeftStickAngle + Math.PI / 4 + currentRobotAngle) * -1;
            leftBackPower = Math.sin(gamepad1LeftStickAngle + Math.PI / 4 + currentRobotAngle);
            rightFrontPower = Math.sin(gamepad1LeftStickAngle + Math.PI / 4 + currentRobotAngle) * -1;
            rightBackPower = Math.cos(gamepad1LeftStickAngle + Math.PI / 4 + currentRobotAngle);

            /*Uses the Y of the right stick to determine the speed of the robot's movement with 0
            being 0.5 power*/
            leftFrontPower *= (-1 * gamepad1RightStickY + 1) / 2;
            leftBackPower *= (-1 * gamepad1RightStickY + 1) / 2;
            rightFrontPower *= (-1 * gamepad1RightStickY + 1) / 2;
            rightBackPower *= (-1 * gamepad1RightStickY + 1) / 2;
        } else {
            leftFrontPower = 0.0;
            leftBackPower = 0.0;
            rightFrontPower = 0.0;
            rightBackPower = 0.0;
        }

        /*Turning using exponential controls so that 50% of the trigger only controls 25% of the
        power while the other 50% controls the other 75%*/
        if (gamepad1LeftTrigger >= TOLERANCE) {
            leftFrontPower += Math.pow(gamepad1LeftTrigger, 2);
            leftBackPower += Math.pow(gamepad1LeftTrigger, 2);
            rightFrontPower += Math.pow(gamepad1LeftTrigger, 2);
            rightBackPower += Math.pow(gamepad1LeftTrigger, 2);
        }
        if (gamepad1RightTrigger >= TOLERANCE) {
            leftFrontPower -= Math.pow(gamepad1RightTrigger, 2);
            leftBackPower -= Math.pow(gamepad1RightTrigger, 2);
            rightFrontPower -= Math.pow(gamepad1RightTrigger, 2);
            rightBackPower -= Math.pow(gamepad1RightTrigger, 2);
        }

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);

    }

    private void funnel() {

        /*Toggle for the collection funnel, when you press the a button the funnel arms either go up
        or down*/
        if (gamepad2.a) {
            if (!aPressed) {
                aPressed = true;
                funnelDown = !funnelDown;
            }
        } else {
            aPressed = false;
        }

        //Setting the funnels to the down position
        if (funnelDown) {
            funnelLeft.setPosition(FUNNEL_LEFT_DOWN);
            funnelRight.setPosition(FUNNEL_RIGHT_DOWN);
        } else {
            funnelLeft.setPosition(FUNNEL_LEFT_UP);
            funnelRight.setPosition(FUNNEL_RIGHT_UP);
        }

    }

    private void bristles() {

        /*Double toggle for the bristles, when you press the b button the bristles spin out,
        when you press the x button they spin in, and when you press the most recent button again,
        they stop*/
        if (gamepad2.x) {
            if (!xPressed) {
                xPressed = true;
                bristlesIn = !bristlesIn;
                if (bristlesIn) {
                    bristlesOut = false;
                }
            }
        } else {
            xPressed = false;
        }
        if (gamepad2.b) {
            if (!bPressed) {
                bPressed = true;
                bristlesOut = !bristlesOut;
                if (bristlesOut) {
                    bristlesIn = false;
                }
            }
        } else {
            bPressed = false;
        }
        if (gamepad2.right_stick_button) {
            if (!rightStickPressed) {
                rightStickPressed = true;
                collectionMaxPower = !collectionMaxPower;
            }
        } else {
            rightStickPressed = false;
        }

        if (bristlesIn) {
            if(collectionMaxPower){
                bristles.setPower(1.0);
            } else {
                bristles.setPower(BRISTLES_DEFAULT_POWER);
            }
            //conveyor.setPosition(0.0); Not needed for first competition!
        } else if (bristlesOut) {
            bristles.setPower(BRISTLES_DEFAULT_POWER * -1);
            //conveyor.setPosition(1.0); Not needed for first competition!
        } else {
            bristles.setPower(0.0);
            //conveyor.setPosition(0.5); Not needed for first competition!
        }

    }

    private void elevator() {

        /*Cycle for the elevator, when you press the right bumper the elevator goes up by one
        position unless it is at the top in which case it loops back to the bottom. When you press
        the left bumper the elevator goes goes down by one position as long as it is not at the
        bottom.*/
        if (gamepad2.right_bumper) {
            if (!rightBumperPressed) {
                rightBumperPressed = true;
                elevatorPositionIndex = (byte)((elevatorPositionIndex + 1) % 5);
            }
        } else {
            rightBumperPressed = false;
        }
        if (gamepad2.left_bumper) {
            if (!leftBumperPressed) {
                leftBumperPressed = true;
                if (elevatorPositionIndex > 0) {
                    elevatorPositionIndex -= 1;
                }
            }
        } else {
            leftBumperPressed = false;
        }

        if(bristlesIn) {
            elevatorPositionIndex = 0;
        }

        switch(elevatorPositionIndex) {
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
                break;
        }

    }

    private void kicker() {

        double gamepad2RightTrigger = gamepad2.right_trigger;

        double kickerPosition;

        if(gamepad2RightTrigger > TOLERANCE) {
            kickerPosition = 1.0 - (gamepad2RightTrigger * KICKER_REDUCTION);
        } else {
            kickerPosition = 1.0;
        }

        kicker.setPosition(kickerPosition);

    }

    private void shooterLifter() {

        double gamepad2LeftStickY = gamepad2.left_stick_y;

        if (Math.abs(gamepad2LeftStickY) > TOLERANCE) {
            shooterLifterTargetPosition -= gamepad2LeftStickY * SHOOTER_LIFTER_REDUCTION;
            if (shooterLifterTargetPosition > SHOOTER_LIFTER_MAX_POSITION) {
                shooterLifterTargetPosition = SHOOTER_LIFTER_MAX_POSITION;
            }
            if (shooterLifterTargetPosition < SHOOTER_LIFTER_MIN_POSITION) {
                shooterLifterTargetPosition = SHOOTER_LIFTER_MIN_POSITION;
            }
        }

        shooterLifterLeft.setPosition(shooterLifterTargetPosition);
        shooterLifterRight.setPosition(shooterLifterTargetPosition);

    }

    private void shooterWheel() {

        /*Toggle for the shooter wheel, when you press the y button it spins counterclockwise when
        you press it again it stops*/
        if (gamepad2.y) {
            if (!yPressed) {
                yPressed = true;
                shooterRev = !shooterRev;
            }
        } else {
            yPressed = false;
        }

        if (shooterRev) {
            shooter.setPower(SHOOTER_POWER * -1);
        } else {
            shooter.setPower(0.0);
        }

    }

}