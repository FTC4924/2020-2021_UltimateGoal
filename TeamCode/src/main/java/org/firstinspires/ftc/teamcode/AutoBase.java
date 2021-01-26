package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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


/**
 * Created by Brendan Clark on 09/24/2020 at 11:51 AM.
 */

public abstract class AutoBase extends OpMode {

    boolean angleSet;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private Servo elevator;
    private DcMotor shooter;

    private BNO055IMU imu;

    private Orientation angles;
    private double currentRobotAngle;

    private double targetAngle;
    private double robotAngleError;

    ArrayList<Command> commands;
    org.firstinspires.ftc.teamcode.Command currentCommand;
    int commandIndex;

    private boolean targetVisible;

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

        angleSet = false;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        elevator = hardwareMap.get(Servo.class, "elevator");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        elevator.setPosition(ElevatorPositions.MIDDLE.positionValue);

        angles = null;
        currentRobotAngle = 0.0;
        targetAngle = 0;

        commands = getCommands();
        currentCommand = commands.get(0);
        commandIndex = 0;

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

    public void start() {
        resetStartTime();
    }

    public void loop() {
        double leftFrontPower = 0;
        double leftBackPower = 0;
        double rightFrontPower = 0;
        double rightBackPower = 0;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, RADIANS);
        currentRobotAngle = angles.firstAngle;

        robotAngleError = targetAngle - currentRobotAngle;
        robotAngleError = ((((robotAngleError - Math.PI) % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI)) - Math.PI;

        telemetry.addData("robot Angle", Math.toDegrees(currentRobotAngle));
        telemetry.addData("error", robotAngleError);

        switch (currentCommand.commandType) {

            case MOVE:

                /*Holonomic controls according to what direction the robot is facing when we start the
                program*/
                if (time < currentCommand.duration) {

                    /*Determines what power each wheel should get based on the angle we get from the stick
                    plus the current robot angle so that the controls are independent of what direction the
                    robot is facing*/
                    leftFrontPower = Math.cos(currentCommand.angle + (Math.PI/4) - currentRobotAngle)*-1;
                    leftBackPower = Math.sin(currentCommand.angle + (Math.PI/4) - currentRobotAngle)*-1;
                    rightFrontPower = Math.sin(currentCommand.angle + (Math.PI/4) - currentRobotAngle);
                    rightBackPower = Math.cos(currentCommand.angle + (Math.PI/4) - currentRobotAngle);

                    // Adjusts motor speed.
                    leftFrontPower *= currentCommand.power;
                    leftBackPower *= currentCommand.power;
                    rightFrontPower *= currentCommand.power;
                    rightBackPower *= currentCommand.power;
                } else {
                    startNextCommand();
                }
                break;

            case TURN:
                if(!angleSet) {
                    angleSet = true;
                    targetAngle = currentCommand.angle;
                    robotAngleError = targetAngle - currentRobotAngle;
                    robotAngleError = ((((robotAngleError - Math.PI) % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI)) - Math.PI;
                }
                if(Math.abs(robotAngleError) < 0.05) {
                    startNextCommand();
                }
                break;

            case AIM:
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
                    telemetry.addData("Robot Angle Error", robotAngleError);
                }

                if(!angleSet) {
                    angleSet = true;
                    robotAngleError = targetAngle - currentRobotAngle;
                }

                if(time > currentCommand.duration) {
                    startNextCommand();
                }
                break;

            case ELEVATOR:
                elevator.setPosition(currentCommand.elevatorPosition.positionValue);
                startNextCommand();
                break;

            case SHOOTER_REV:
                shooter.setPower(currentCommand.power * -1);
                startNextCommand();
                break;

            case WAIT:
                if(time > currentCommand.duration) {
                    startNextCommand();
                }
                break;
        }
        telemetry.addData("command",currentCommand.commandType);

        leftFrontPower += robotAngleError * 2;
        leftBackPower += robotAngleError * 2;
        rightFrontPower += robotAngleError * 2;
        rightBackPower += robotAngleError * 2;

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }


    public void startNextCommand() {
        commandIndex ++;
        if (commandIndex < commands.size()) {
            currentCommand = commands.get(commandIndex);
            angleSet = false;
            resetStartTime();

        }
    }

    public abstract ArrayList<Command> getCommands();

}
