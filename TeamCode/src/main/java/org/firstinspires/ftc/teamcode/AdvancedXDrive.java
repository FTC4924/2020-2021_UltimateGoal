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

    //Declaring all variables and objects in the program
    //Reused analog values from the controller
    private double gamepad1LeftStickX;
    private double gamepad1LeftStickY;
    private double gamepad1RightStickY;
    private double gamepad1LeftTrigger;
    private double gamepad1RightTrigger;
    private double gamepad2LeftStickY;

    //Reused Calculated Values
    private double gamepad1LeftStickAngle;

    private double shooterTargetPosition;

    //Logic
    private double leftFrontPower;
    private double leftBackPower;
    private double rightFrontPower;
    private double rightBackPower;

    private boolean bPressed;
    private boolean xPressed;
    private boolean bristlesIn;
    private boolean bristlesOut;

    private boolean rightBumperPressed;
    private boolean leftBumperPressed;
    private int elevatorPositionIndex;

    private boolean yPressed;
    private boolean shooterRev;

    private double kickerPosition;

    private boolean aPressed;
    private boolean funnelDown;

    //Motors and servos
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;
    public DcMotor bristles;
    public Servo elevator;
    public Servo kicker;
    public DcMotor shooter;
    public Servo funnelLeft;
    public Servo funnelRight;
    public Servo shooterLeft;
    public Servo shooterRight;


    //creating the variables for the gyro sensor
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;

    Orientation angles;
    private double currentRobotAngle;
    private double angleOffset;

    public void init() {

        //Initializing all of the variables to their default values
        gamepad1LeftStickX = 0.0;
        gamepad1LeftStickY = 0.0;
        gamepad1RightStickY = 0.0;
        gamepad1RightTrigger = 0.0;
        gamepad1LeftTrigger = 0.0;
        gamepad2LeftStickY = 0.0;

        gamepad1LeftStickAngle = 0.0;

        shooterTargetPosition = SHOOTER_DEFAULT_POSITION;

        leftFrontPower = 0.0;
        leftBackPower = 0.0;
        rightFrontPower = 0.0;
        rightBackPower = 0.0;

        currentRobotAngle = 0.0;
        angleOffset = 0.0;

        bPressed = false;
        xPressed = false;
        bristlesIn = false;
        bristlesOut = false;

        rightBumperPressed = false;
        leftBumperPressed = false;
        elevatorPositionIndex = 0;

        yPressed = false;
        shooterRev = false;

        kickerPosition = 0.0;

        //Instantiating the motor and servo objects as their appropriate motor/servo in the configuration on the robot
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        bristles = hardwareMap.get(DcMotor.class, "collection");
        elevator = hardwareMap.get(Servo.class, "elevator");
        kicker = hardwareMap.get(Servo.class, "kicker");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        shooterLeft = hardwareMap.get(Servo.class, "shooterLeft");
        shooterRight = hardwareMap.get(Servo.class, "shooterRight");
        /*
        funnelLeft = hardwareMap.get(Servo.class, "funnelLeft");
        funnelRight = hardwareMap.get(Servo.class, "funnelRight");
         */


        //Setting the shooter motor to brake rather than drift when the power is set to 0
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  //Changed to FLOAT on 12/24/2020 because I don't want to break our motor (E)

        //Instantiating objects and initializing variables for the gyro sensor
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

        telemetry.addData("angleOffset", angleOffset);

        //getting the angle of the robot from the IMU
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);
        currentRobotAngle = angles.firstAngle - angleOffset;

        if(gamepad1.b) {
            angleOffset = angles.firstAngle;
        }

        //Setting the reused analog values on the controller to more easily accessible variables
        gamepad1LeftStickX = gamepad1.left_stick_x;
        gamepad1LeftStickY = gamepad1.left_stick_y;
        gamepad2LeftStickY = gamepad2.left_stick_y;
        gamepad1RightStickY = gamepad1.right_stick_y;
        gamepad1LeftTrigger = gamepad1.left_trigger;
        gamepad1RightTrigger = gamepad1.right_trigger;

        //Holonomic controls according to what direction the robot is facing when we start the program
        if(Math.abs(gamepad1LeftStickX) >= TOLERANCE || Math.abs(gamepad1LeftStickY) >= TOLERANCE) {
            //Uses atan2 to convert the x and y values of the controller to an angle
            gamepad1LeftStickAngle = Math.atan2(gamepad1LeftStickY, gamepad1LeftStickX);

            /*Determines what power each wheel should get based on the angle we get from the stick plus the
            current robot angle so that the controls are independent of what direction the robot is facing*/
            leftFrontPower = Math.cos(gamepad1LeftStickAngle + Math.toRadians(currentRobotAngle) + Math.PI / 4) * -1;
            leftBackPower = Math.sin((gamepad1LeftStickAngle + (Math.PI / 4)) + Math.toRadians(currentRobotAngle));
            rightFrontPower = Math.sin(gamepad1LeftStickAngle + Math.toRadians(currentRobotAngle) + Math.PI / 4) * -1;
            rightBackPower = Math.cos((gamepad1LeftStickAngle + (Math.PI / 4)) + Math.toRadians(currentRobotAngle));

            //Uses the Y of the right stick to determine the speed of the robot's movement with 0 being 0.5 power
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

        /*Rudimentary turning using exponential controls so that 50% of the trigger only controls 25% of the
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

        /*Double toggle for the bristles, when you press the b button the bristles spin out,
        when you press the x button they spin in, and when you press the most recent one again it stops*/
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

        /*Cycle for the elevator, when you press the right bumper elevator index goes up by 1 unless it
        goes above 4 in which case it loops back to 0. When you press the left bumper elevator index goes
        down as long as it is greater than 0*/
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
                    elevatorPositionIndex -= 1;
                }
            }
        } else {
            leftBumperPressed = false;
        }

        //Toggle for the shooter wheel, when you press the y button it spins counterclockwise when you press it again it stops
        if (gamepad2.y) {
            if (!yPressed) {
                yPressed = true;
                shooterRev = !shooterRev;
            }
        } else {
            yPressed = false;
        }

        if(gamepad2.right_trigger > TOLERANCE) {
            kickerPosition = 1.0 - (gamepad2.right_trigger * KICKER_REDUCTION); //gives some control over how far forward the kicker is
            //integer at the end corresponds to how many degrees we turn (1 = 180, 0.5 = 90, 0.25 = 45)
        } else {
            kickerPosition = 1.0; //kicker out of the way/full back in poistion 1
        }

        //Toggle for the collection funnel, when you press the a button the funnels go either down or up
        if (gamepad2.a) {
            if (!aPressed) {
                aPressed = true;
                funnelDown = !funnelDown;
            }
        } else {
            aPressed = false;
        }

        //Manual aim for the shooter
        if (Math.abs(gamepad2LeftStickY) > TOLERANCE) {
            shooterTargetPosition -= gamepad2LeftStickY / SHOOTER_MANUAL_REDUCTION;
            if (shooterTargetPosition > 1.0) {
                shooterTargetPosition = 1.0;
            }
            if (shooterTargetPosition < 0.0) {
                shooterTargetPosition = 0.0;
            }
        }

        //Setting the power of the wheels based on the calculations above
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);

        //determine how the bristles rotate based on the logic above
        if (bristlesIn) {
            bristles.setPower(BRISTLES_POWER);
        } else if (bristlesOut) {
            bristles.setPower(BRISTLES_POWER * -1);
        } else {
            bristles.setPower(0.0);
        }

        //Setting the elevator to a position based on elevatorPositionIndex determined above
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

        /*
        //Setting the aiming servos to shooterTargetPosition
        shooterLeft.setPosition(shooterTargetPosition);
        shooterRight.setPosition(shooterTargetPosition);
         */

        //Spin the shooter wheel or not based on the logic above
        if (shooterRev) {
            shooter.setPower(SHOOTER_POWER * -1);
        } else {
            shooter.setPower(0.0);
        }

        kicker.setPosition(kickerPosition);

        /*
        //Setting the funnels to the down position
        if (funnelDown) {
            funnelLeft.setPosition(FUNNEL_LEFT_DOWN);
            funnelRight.setPosition(FUNNEL_RIGHT_DOWN);
        } else {
            funnelLeft.setPosition(FUNNEL_LEFT_UP);
            funnelRight.setPosition(FUNNEL_RIGHT_UP);
        }
        */
    }
}