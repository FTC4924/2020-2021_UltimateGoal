package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
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
    //private Servo conveyor;


    //Creating the variables for the gyro sensor
    private BNO055IMU imu;

    private Orientation angles;
    private double angleOffset;
    private double currentRobotAngle;
    private double targetAngle;
    private boolean targetVisible;
    private double robotAngleError;

    private OpenGLMatrix robotFromCamera;
    private OpenGLMatrix robotLocationTransform;
    private OpenGLMatrix lastLocation;
    private VectorF robotPosition;
    private Orientation robotRotation;

    WebcamName webcam1;

    int cameraMonitorViewId;
    VuforiaLocalizer.Parameters vuforiaParameters;
    private VuforiaLocalizer vuforia;

    VuforiaTrackables targetsUltimateGoal;

    VuforiaTrackable blueTowerGoalTarget;
    VuforiaTrackable redTowerGoalTarget;
    VuforiaTrackable redAllianceTarget;
    VuforiaTrackable blueAllianceTarget;
    VuforiaTrackable frontWallTarget;

    List<VuforiaTrackable> allTrackables;

    public void init() {

        angleOffset = -1 * Math.PI / 2;
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

        //Initializing the Revhub IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = null;
        currentRobotAngle = 0.0;

        robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, RADIANS, CAMERA_X_ROTATE, CAMERA_Y_ROTATE, CAMERA_Z_ROTATE));
        robotLocationTransform = null;
        lastLocation = null;
        robotPosition = null;
        robotRotation = null;

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        vuforiaParameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuforiaParameters.cameraName = webcam1;

        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParameters);

        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");

        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, RADIANS, 0, 0 , 0)));

        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, RADIANS, 0, 0, 0)));

        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(robotFromCamera, vuforiaParameters.cameraDirection);
        }

        targetsUltimateGoal.activate();

    }

    public void loop() {

        telemetry.addData("angleOffset", angleOffset);

        //Getting the angle of the robot from the IMU
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

    /**
     * Sets the gyro angle offset based off of the current angle when the b button is pressed.
     */
    private void recalibrateGyro() {
        if(gamepad1.b) {
            angleOffset = angles.firstAngle;
        }

    }

    /**
     * Holonomic controls according to what direction the robot is facing when we start the
     * program or when we recalibrate the gyro.
     * Uses the left stick to control movement, the triggers to control turning using exponential
     * controls, and the right stick up and down for speed.
     */
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

        if (gamepad1.dpad_up) {
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    //telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {

                        lastLocation = robotLocationTransform;

                    }

                    break;

                }
            }

            if (targetVisible) {
                robotPosition = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        robotPosition.get(0), robotPosition.get(1), robotPosition.get(2));
                telemetry.addData("TargetAngle", Math.toDegrees(targetAngle));

                robotRotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, RADIANS);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", robotRotation.firstAngle, robotRotation.secondAngle, robotRotation.thirdAngle);

                targetAngle = -1 * Math.atan((robotPosition.get(2) - 264.4) / 1828.8);
                telemetry.addData("Needed Robot Angle", targetAngle);
                robotAngleError = targetAngle - angles.firstAngle;
                telemetry.addData("Robot Angle Error", robotAngleError);

                //leftFrontPower += robotAngleError;
                //leftBackPower += robotAngleError;
                //rightFrontPower += robotAngleError;
                //rightBackPower += robotAngleError;

                /*distanceFromImage = Math.sqrt(Math.pow(robotPosition.get(0),2) + Math.pow(robotPosition.get(2),2));

                for (int i = 0; i < aimingLookupTable.length - 1; i++) {
                    if (distanceFromImage >= aimingLookupTable[i][0] && distanceFromImage <= aimingLookupTable[i + 1][0]) {

                        lowerLookupTableIndex = i;

                        break;

                    }
                }

                verticalAngleSlope = (aimingLookupTable[lowerLookupTableIndex + 1][1] - aimingLookupTable[lowerLookupTableIndex][1] / (aimingLookupTable[lowerLookupTableIndex + 1][0] - aimingLookupTable[lowerLookupTableIndex][0]));
                verticalAngleYIntercept = aimingLookupTable[lowerLookupTableIndex][1] - verticalAngleSlope * aimingLookupTable[lowerLookupTableIndex][0];
                neededVerticalAngle = (int) Math.round(verticalAngleSlope*distanceFromImage+verticalAngleYIntercept);
                telemetry.addData("Needed Vertical Angle", neededVerticalAngle);*/

                //telemetry.addData("Needed Cam Position", neededCamPosition);
                //camMotor.setTargetPosition(neededCamPosition);

                //telemetry.addData("Current Cam Motor Target Position", camMotor.getTargetPosition());
            }
        }

        //Sets the wheel powers
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);

    }

    /**
     * Toggle for the collection funnel, when you press the a button the funnel arms either go up
     * or down.
     */
    private void funnel() {

        //Toggle for the collection funnel
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

    /**
     * Double toggle for the bristles, when you press the button the bristles spin out,
     * when you press the x button they spin in, and when you press the most recent button again,
     * they stop.
     */
    private void bristles() {

        //Double toggle for the bristles
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

        //Setting the bristles power
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

    /**
     * Cycle for the elevator, when you press the right bumper the elevator goes up by one
     * position unless it is at the top in which case it loops back to the bottom. When you press
     * the left bumper the elevator goes goes down by one position as long as it is not at the
     * bottom.
     */
    private void elevator() {

        //Cycle for the elevator
        if (gamepad2.right_bumper) {
            if (!rightBumperPressed) {
                rightBumperPressed = true;
                elevatorPositionIndex = (byte)((elevatorPositionIndex + 1) % 6);
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

        //Setting the elevator position
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
            case 5:
                elevator.setPosition(ElevatorPositions.UNJAM.positionValue);
                break;
        }

    }

    /**
     * Triggers the kicker when the right trigger is pressed.
     */
    private void kicker() {

        double gamepad2RightTrigger = gamepad2.right_trigger;
        double kickerPosition;

        //Calculates the kicker position
        if(gamepad2RightTrigger > TOLERANCE) {
            kickerPosition = 1.0 - (gamepad2RightTrigger * KICKER_REDUCTION);
        } else {
            kickerPosition = 1.0;
        }

        //Sets the kicker position.
        kicker.setPosition(kickerPosition);

    }

    /**
     * Manual aiming for the shooter using the left stick up and down.
     */
    private void shooterLifter() {

        double gamepad2LeftStickY = gamepad2.left_stick_y;

        //Calculates the shooter target position.
        if (Math.abs(gamepad2LeftStickY) > TOLERANCE) {
            shooterLifterTargetPosition -= gamepad2LeftStickY * SHOOTER_LIFTER_REDUCTION;
            if (shooterLifterTargetPosition > SHOOTER_LIFTER_MAX_POSITION) {
                shooterLifterTargetPosition = SHOOTER_LIFTER_MAX_POSITION;
            }
            if (shooterLifterTargetPosition < SHOOTER_LIFTER_MIN_POSITION) {
                shooterLifterTargetPosition = SHOOTER_LIFTER_MIN_POSITION;
            }
        }

        //Sets the shooter lifter positions.
        shooterLifterLeft.setPosition(shooterLifterTargetPosition);
        shooterLifterRight.setPosition(shooterLifterTargetPosition);

    }

    /**
     * Toggle for the shooter wheel, when you press the y button it spins counter clockwise, and
     * when you press it again it stops.
     */
    private void shooterWheel() {

        //Toggle for the shooter wheel
        if (gamepad2.y) {
            if (!yPressed) {
                yPressed = true;
                shooterRev = !shooterRev;
            }
        } else {
            yPressed = false;
        }

        //Sets the shooter to on or off.
        if (shooterRev) {
            shooter.setPower(SHOOTER_POWER * -1);
        } else {
            shooter.setPower(0.0);
        }

    }

}