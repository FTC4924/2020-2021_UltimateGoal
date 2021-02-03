package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.teamcode.Constants.VUFORIA_KEY;

@TeleOp(name="VuforiaSteering")
public class VuforiaSteering extends OpMode {

    private static final float MM_Per_Inch = 25.4f;

    /*
    public static final double robotAngleKP = .005;
    public static final double robotAngleKI = 0;
    public static final double robotAngleKD = 0;
     */

    private static final float CAMERA_FORWARD_DISPLACEMENT = 0;
    private static final float CAMERA_VERTICAL_DISPLACEMENT = 0;
    private static final float CAMERA_LEFT_DISPLACEMENT = 0;
    private static final float CAMERA_X_ROTATE = 0;
    private static final float CAMERA_Y_ROTATE = 0;
    private static final float CAMERA_Z_ROTATE = 0;

    private double gamepad1LeftStickY;
    private double gamepad1LeftStickX;
    private double gamepad1RightStickX;
    private double gamepad1RightStickY;

    private double middlePower;
    private double leftPower;
    private double rightPower;

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

    /*
    public double lastRobotAngleError;
    public double robotAngleIntegral;
    public double robotAngleDerivative;
     */

    private DcMotor rightMotor;
    private DcMotor leftMotor;
    private DcMotor middleMotor;

    private DcMotor camMotor;

    BNO055IMU imu;
    Orientation angles;
    BNO055IMU.Parameters parameters;

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

        middlePower = 0.0;
        leftPower = 0.0;
        rightPower = 0.0;

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

        middleMotor = hardwareMap.get(DcMotor.class, "middleWheelMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        camMotor = hardwareMap.get(DcMotor.class, "camMotor");
        camMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        camMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        camMotor.setTargetPosition(0);
        camMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        angles = null;

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
        gamepad1RightStickX = gamepad1.right_stick_x / 2;
        gamepad1RightStickY = gamepad1.right_stick_y;

        middlePower = 0;
        leftPower = 0;
        rightPower = 0;

        targetVisible = false;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);;

        if (gamepad1LeftStickX > 0.05 || gamepad1LeftStickX < -.05) {

            middlePower = gamepad1LeftStickX;
        }

        if (gamepad1LeftStickY > 0.05 || gamepad1LeftStickY < -.05) {

            leftPower = gamepad1LeftStickY;
            rightPower = gamepad1LeftStickY;

        }

        if (gamepad1RightStickX > 0.05) {
            if (leftPower > 1 - gamepad1RightStickX) {

                leftPower -= rightPower + gamepad1RightStickX - 1 + gamepad1RightStickX;
                rightPower = 1;

            } else if (leftPower < -1 + gamepad1RightStickX) {

                rightPower -= leftPower - gamepad1RightStickX + 1 - gamepad1RightStickX;
                leftPower = -1;

            } else {

                rightPower += gamepad1RightStickX;
                leftPower -= gamepad1RightStickX;

            }
        } else if (gamepad1RightStickX < -0.05) {
            if (leftPower > 1 + gamepad1RightStickX) {

                rightPower -= leftPower - gamepad1RightStickX - 1 - gamepad1RightStickX;
                leftPower = 1;

            } else if (leftPower < -1 - gamepad1RightStickX) {

                leftPower -= rightPower + gamepad1RightStickX + 1 + gamepad1RightStickX;
                rightPower = -1;

            } else {

                leftPower -= gamepad1RightStickX;
                rightPower += gamepad1RightStickX;

            }
        }

        if (gamepad2.left_stick_y > 0.01 || gamepad2.left_stick_y < -0.01) {

            camMotor.setTargetPosition((int) Math.round(camMotor.getCurrentPosition() + gamepad2.left_stick_y/3));

        }

        if (gamepad1.b) {

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
                        robotPosition.get(0) / MM_Per_Inch, robotPosition.get(1) / MM_Per_Inch, robotPosition.get(2) / MM_Per_Inch);

                robotRotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", robotRotation.firstAngle, robotRotation.secondAngle, robotRotation.thirdAngle);

                currentRobotAngle = angles.firstAngle;
                neededRobotAngle = Math.toDegrees(Math.atan(robotPosition.get(0) / robotPosition.get(2)));
                telemetry.addData("Needed Robot Angle", neededRobotAngle);
                robotAngleError = currentRobotAngle - neededRobotAngle;
                telemetry.addData("Robot Angle Error", robotAngleError);


                if (robotAngleError > 5) {

                    rightPower = 0.2;
                    leftPower = -0.2;

                } else if (robotAngleError < -5) {

                    rightPower = -0.2;
                    leftPower = 0.2;

                } else {

                    rightPower = 0.0;
                    leftPower = 0.0;

                }

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

        middleMotor.setPower(middlePower);
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        telemetry.update();
    }
}
