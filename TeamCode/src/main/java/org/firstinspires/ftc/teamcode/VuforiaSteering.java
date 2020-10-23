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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name="VuforiaSteering")
public class VuforiaSteering extends OpMode {

    int camPosition;
    int neededPosition;

    WebcamName webcamName = null;

    BNO055IMU imu;
    Orientation angles;
    BNO055IMU.Parameters parameters;

    private DcMotor rightMotor;
    private DcMotor leftMotor;
    private DcMotor middleMotor;

    private DcMotor camMotor;

    public double neededAngle;
    public double currentAngle;

    public double error;
    public double lastError;
    public double integral;
    public double derivative;

    public static final double KP = .005;
    public static final double KI = 0;
    public static final double KD = 0;

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_LANDSCAPE = true;

    private static final String VUFORIA_KEY = "AaeF/Hb/////AAABmXyUA/dvl08Hn6O8IUco1axEjiRtYCVASeXGzCnFiMaizR1b3cvD+SXpU1UHHbSpnyem0dMfGb6wce32IWKttH90xMTnLjY4aXBEYscpQbX/FzUi6uf5M+sXDVNMtaVxLDGOb1phJ8tg9/Udb1cxIUCifI+AHmcwj3eknyY1ZapF81n/R0mVSmuyApS2oGQLnETWaWK+kxkx8cGnQ0Nj7a79gStXqm97obOdzptw7PdDNqOfSLVcyKCegEO0zbGoInhRMDm0MPPTxwnBihZsjDuz+I5kDEZJZfBWZ9O1PZMeFmhe6O8oFwE07nFVoclw7j2P6qHbsKTabg3w9w4ZdeTSZI4sV2t9OhbF13e0MWeV";

    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;

    private static final float stoneZ = 2.00f * mmPerInch;

    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;
    private static final float bridgeRotZ = 180;

    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    OpenGLMatrix robotFromCamera;

    final float CAMERA_FORWARD_DISPLACEMENT  = 0;
    final float CAMERA_VERTICAL_DISPLACEMENT = 0;
    final float CAMERA_LEFT_DISPLACEMENT     = 0;

    private OpenGLMatrix lastLocation;
    private VuforiaLocalizer vuforia;
    private boolean targetVisible;
    private float phoneXRotate;
    private float phoneYRotate;
    private float phoneZRotate;

    int cameraMonitorViewId;
    VuforiaLocalizer.Parameters vuforiaParameters;

    VuforiaTrackables targetsUltimateGoal;

    VuforiaTrackable blueTowerGoalTarget;
    VuforiaTrackable redTowerGoalTarget;
    VuforiaTrackable redAllianceTarget;
    VuforiaTrackable blueAllianceTarget;
    VuforiaTrackable frontWallTarget;

    private double middlePower;
    private double leftPower;
    private double rightPower;

    private double gamepad1LeftStickY;
    private double gamepad1LeftStickX;
    private double gamepad1RightStickX;
    private double gamepad1RightStickY;


    List<VuforiaTrackable> allTrackables;

    public void init() {

        middlePower = 0;
        leftPower = 0;
        rightPower = 0;

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        middleMotor = hardwareMap.get(DcMotor.class, "middleWheelMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        camMotor = hardwareMap.get(DcMotor.class, "camMotor");
        camMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        camMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        camMotor.setTargetPosition(0);
        camMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        camMotor.setPower(.5);

        parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        lastLocation = null;
        targetVisible = false;
        phoneXRotate = 0;
        phoneYRotate = 0;
        phoneZRotate = 0;

        /*if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }*/

        if (PHONE_IS_LANDSCAPE) {
            phoneXRotate = 90 ;
        }

        robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        vuforiaParameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuforiaParameters.cameraDirection = CAMERA_CHOICE;
        vuforiaParameters.cameraName = webcamName;

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

        middlePower = 0;
        leftPower = 0;
        rightPower = 0;

        camPosition = camMotor.getCurrentPosition();
        telemetry.addData("Cam Position", camPosition);

        /*if (gamepad2.left_stick_y > 0.01 || gamepad2.left_stick_y < -0.01) {

            camMotor.setPower(gamepad2.left_stick_y/3);

        } else {

            camMotor.setPower(0.0);

        }*/

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentAngle = angles.firstAngle;

        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                //telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        if (targetVisible) {

            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            telemetry.addData("Needed Angle", Math.toDegrees(Math.atan(translation.get(0) / translation.get(2))));
            neededAngle = Math.toDegrees(Math.atan(translation.get(0) / translation.get(2)));
            error = currentAngle - neededAngle;

            telemetry.addData("Error Value", error);

            if (gamepad2.b) {

                if (Math.abs(error) > 3) {

                    rightPower = error * 1/70;
                    leftPower = -error * 1/70;

                } else {

                    rightPower = 0.0;
                    leftPower = 0.0;
                }
            }

            neededAngle = Math.toDegrees(Math.atan(35/Math.sqrt(Math.pow(translation.get(0),2) + Math.pow(translation.get(2),2))));
            neededPosition = (int)(neededAngle * 21.2 - 8);
            telemetry.addData("Needed Cam Angle", neededAngle);
            telemetry.addData("Needed Cam Position", neededPosition);
            if(gamepad2.a){
                camMotor.setTargetPosition(neededPosition);
            }
            telemetry.addData("Cam Motor Target Position", camMotor.getTargetPosition());
        }
        else {
            //telemetry.addData("Visible Target", "none");
            leftPower = 0.0;
            rightPower = 0.0;
        }

        gamepad1LeftStickX = gamepad1.left_stick_x;
        gamepad1LeftStickY = gamepad1.left_stick_y;
        gamepad1RightStickX = gamepad1.right_stick_x / 2;
        gamepad1RightStickY = gamepad1.right_stick_y;

        if(Math.abs(gamepad1LeftStickX + gamepad1LeftStickY + gamepad1RightStickX + gamepad1RightStickY) > 0.05) {
            middlePower = 0;
            leftPower = 0;
            rightPower = 0;
        }

        if(gamepad1LeftStickX > 0.05 || gamepad1LeftStickX < -.05) {

            middlePower = gamepad1LeftStickX;
        }

        if(gamepad1LeftStickY > 0.05 || gamepad1LeftStickY < -.05) {

            leftPower = gamepad1LeftStickY;
            rightPower = gamepad1LeftStickY;

        }

        if(gamepad1RightStickX > 0.05){
            if(leftPower > 1 - gamepad1RightStickX) {

                leftPower -= rightPower + gamepad1RightStickX - 1 + gamepad1RightStickX;
                rightPower = 1;

            } else if(leftPower < -1 + gamepad1RightStickX) {

                rightPower -= leftPower - gamepad1RightStickX + 1 - gamepad1RightStickX;
                leftPower = -1;

            } else {

                rightPower += gamepad1RightStickX;
                leftPower -= gamepad1RightStickX;

            }
        } else if(gamepad1RightStickX < -0.05) {
            if(leftPower > 1 + gamepad1RightStickX) {

                rightPower -= leftPower - gamepad1RightStickX - 1 - gamepad1RightStickX;
                leftPower = 1;

            } else if(leftPower < -1 - gamepad1RightStickX) {

                leftPower -= rightPower + gamepad1RightStickX + 1 + gamepad1RightStickX;
                rightPower = -1;

            } else {

                leftPower -= gamepad1RightStickX;
                rightPower += gamepad1RightStickX;

            }
        }

        middleMotor.setPower(middlePower);
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        telemetry.update();
    }
}
