package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
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

    private boolean targetVisible;

    private double distanceFromImage;

    private double currentRobotAngle;
    private double neededRobotAngle;
    private double robotAngleError;

    private double neededVerticalAngle;
    private int currentCamPosition;
    private int neededCamPosition;

    private double [][] aimingLookupTable;
    private int lowerLookupTableIndex;
    private double verticalAngleSlope;
    private double verticalAngleYIntercept;

    private static final String VUFORIA_KEY = "AaeF/Hb/////AAABmXyUA/dvl08Hn6O8IUco1axEjiRtYCVASeXGzCnFiMaizR1b3cvD+SXpU1UHHbSpnyem0dMfGb6wce32IWKttH90xMTnLjY4aXBEYscpQbX/FzUi6uf5M+sXDVNMtaVxLDGOb1phJ8tg9/Udb1cxIUCifI+AHmcwj3eknyY1ZapF81n/R0mVSmuyApS2oGQLnETWaWK+kxkx8cGnQ0Nj7a79gStXqm97obOdzptw7PdDNqOfSLVcyKCegEO0zbGoInhRMDm0MPPTxwnBihZsjDuz+I5kDEZJZfBWZ9O1PZMeFmhe6O8oFwE07nFVoclw7j2P6qHbsKTabg3w9w4ZdeTSZI4sV2t9OhbF13e0MWeV";

    private static final float CAMERA_FORWARD_DISPLACEMENT = 0;
    private static final float CAMERA_VERTICAL_DISPLACEMENT = 0;
    private static final float CAMERA_LEFT_DISPLACEMENT = 0;
    private static final float CAMERA_X_ROTATE = 0;
    private static final float CAMERA_Y_ROTATE = 0;
    private static final float CAMERA_Z_ROTATE = 0;

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

        targetVisible = false;

        distanceFromImage = 0.0;

        currentRobotAngle = 0.0;
        neededRobotAngle = 0.0;
        robotAngleError = 0.0;

        neededVerticalAngle = 0.0;
        currentCamPosition = 0;
        neededCamPosition = 0;

        aimingLookupTable = new double[5][2];

        aimingLookupTable[0][0] = 0;
        aimingLookupTable[0][1] = 2;
        aimingLookupTable[1][0] = 1;
        aimingLookupTable[1][1] = 3;
        aimingLookupTable[2][0] = 2;
        aimingLookupTable[2][1] = 4;
        aimingLookupTable[3][0] = 7;
        aimingLookupTable[3][1] = 9;
        aimingLookupTable[4][0] = 7;
        aimingLookupTable[4][1] = 9;

        lowerLookupTableIndex = 0;
        verticalAngleSlope = 0.0;
        verticalAngleYIntercept = 0.0;

        robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, CAMERA_X_ROTATE, CAMERA_Y_ROTATE, CAMERA_Z_ROTATE));
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
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, 0 , 0)));

        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(0, 0, 0)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, 0, 0)));

        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(robotFromCamera, vuforiaParameters.cameraDirection);
        }

        targetsUltimateGoal.activate();

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
