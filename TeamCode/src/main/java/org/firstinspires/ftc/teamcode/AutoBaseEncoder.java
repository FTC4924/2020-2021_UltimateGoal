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

public abstract class AutoBaseEncoder extends OpMode {

    /*
    We cleaned up the code, hopefully fixed vuforia, got the code to use motor encoders regardless of the battery's charge, and modularised some of the code. Changed the Vuforia code to detect the image then turn, rather than detecting the image while turning.
     */

    private ArrayList<Command> commands;
    private Command currentCommand;
    private int commandIndex;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor shooter;

    private Servo elevator;
    private Servo shooterLifterLeft;
    private Servo shooterLifterRight;
    private Servo kicker;

    private double targetPosition;
    private double leftFrontTargetPosition;
    private double leftBackTargetPosition;
    private double rightFrontTargetPosition;
    private double rightBackTargetPosition;

    double leftFrontPower = 0;
    double leftBackPower = 0;
    double rightFrontPower = 0;
    double rightBackPower = 0;

    private BNO055IMU imu;
    private Orientation angles;

    private double currentRobotAngle;
    private double targetAngle;
    private double robotAngleError;
    private boolean onFirstLoop;

    private OpenGLMatrix robotFromCamera;
    private OpenGLMatrix robotLocationTransform;
    private OpenGLMatrix lastLocation;
    private VectorF robotPosition;
    private Orientation robotRotation;

    private WebcamName webcam1;
    private int cameraMonitorViewId;
    private VuforiaLocalizer.Parameters vuforiaParameters;
    private VuforiaLocalizer vuforia;

    private VuforiaTrackables targetsUltimateGoal;

    private VuforiaTrackable blueTowerGoalTarget;
    private VuforiaTrackable redTowerGoalTarget;
    private VuforiaTrackable redAllianceTarget;
    private VuforiaTrackable blueAllianceTarget;
    private VuforiaTrackable frontWallTarget;

    private List<VuforiaTrackable> allTrackables;

    private boolean targetVisible;
    private boolean targetVisible2;

    private double highest = 0;
    private double lowest = 20000000;

    public void init() {

        commands = getCommands();
        currentCommand = commands.get(0);
        commandIndex = 0;

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setTargetPosition(0);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setTargetPosition(0);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setTargetPosition(0);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setTargetPosition(0);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        
        elevator = hardwareMap.get(Servo.class, "elevator");
        elevator.setPosition(ElevatorPositions.MIDDLE.positionValue);
        shooterLifterLeft = hardwareMap.get(Servo.class, "shooterLeft");
        shooterLifterLeft.setPosition(.35);
        shooterLifterRight = hardwareMap.get(Servo.class, "shooterRight");
        shooterLifterRight.setPosition(.35);
        kicker = hardwareMap.get(Servo.class, "kicker");

        targetPosition = 0.0;
        leftFrontTargetPosition = 0.0;
        leftBackTargetPosition = 0.0;
        rightFrontTargetPosition = 0.0;
        rightBackTargetPosition = 0.0;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = null;

        currentRobotAngle = 0.0;
        targetAngle = 0.0;
        robotAngleError = 0.0;
        onFirstLoop = true;

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

        targetVisible = false;
        targetVisible2 = targetVisible;

    }

    public void start() {
        resetStartTime();
    }

    public void loop() {

        leftFrontPower = 0;
        leftBackPower = 0;
        rightFrontPower = 0;
        rightBackPower = 0;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, RADIANS);
        currentRobotAngle = angles.firstAngle;

        robotAngleError = targetAngle - currentRobotAngle;
        robotAngleError = ((((robotAngleError - Math.PI) % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI)) - Math.PI;

        switch (currentCommand.commandType) {

            case MOVE:
                holonomicDrive();
                break;

            case TURN:
                turn();
                break;

            case AIM:
                aim();
                break;

            case ELEVATOR:
                elevator();
                break;

            case SHOOTER_REV:
                shooterRev();
                break;

            case WAIT:
                pause();
                break;

            case KICKER:
                kicker();
                break;
        }

        leftFrontTargetPosition += robotAngleError * 20;
        leftBackTargetPosition += robotAngleError * 20;
        rightFrontTargetPosition += robotAngleError * 20;
        rightBackTargetPosition += robotAngleError * 20;

        leftFrontPower += robotAngleError * 3;
        leftBackPower += robotAngleError * 3;
        rightFrontPower += robotAngleError * 3;
        rightBackPower += robotAngleError * 3;

        // Adjusts motor speed.
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);

        leftFront.setTargetPosition((int) Math.round(leftFrontTargetPosition));
        leftBack.setTargetPosition((int) Math.round(leftBackTargetPosition));
        rightFront.setTargetPosition((int) Math.round(rightFrontTargetPosition));
        rightBack.setTargetPosition((int) Math.round(rightBackTargetPosition));

        telemetry.addData("targetVisible", targetVisible2);

    }

    /**
     * Calculates the holonomic drive motor target encoder positions and sets the motor speeds.
     */
    public void holonomicDrive() {
        /*Determines what encoder position each wheel should be based on the angle we get from the input
        plus the current robot angle so that the controls are independent of what direction the
        robot is facing*/
        if (onFirstLoop) {

            targetPosition = currentCommand.distance * TICKS_PER_FOOT;

            double leftTargetPositionCalculation = targetPosition * Math.cos(currentCommand.angle + (Math.PI / 4) - currentRobotAngle);
            double rightTargetPositionCalculation = targetPosition * Math.sin(currentCommand.angle + (Math.PI / 4) - currentRobotAngle);

            leftFrontTargetPosition += (leftTargetPositionCalculation * -1);
            leftBackTargetPosition += (rightTargetPositionCalculation * -1);
            rightFrontTargetPosition += (rightTargetPositionCalculation);
            rightBackTargetPosition += (leftTargetPositionCalculation);

            onFirstLoop = false;
        }

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

        if (Math.abs(leftFront.getCurrentPosition() - leftFrontTargetPosition) <= ENCODER_TOLERANCE &&
                Math.abs(leftBack.getCurrentPosition() - leftBackTargetPosition) <= ENCODER_TOLERANCE &&
                Math.abs(rightFront.getCurrentPosition() - rightFrontTargetPosition) <= ENCODER_TOLERANCE &&
                Math.abs(rightBack.getCurrentPosition() - rightBackTargetPosition) <= ENCODER_TOLERANCE) {

            startNextCommand();
        }
    }

    /**
     * Turns using the encoder positions.
      */
    public void turn() {
        if(onFirstLoop) {
            onFirstLoop = false;
            targetAngle = currentCommand.angle;
            robotAngleError = targetAngle - currentRobotAngle;
            robotAngleError = ((((robotAngleError - Math.PI) % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI)) - Math.PI;
        }
        if(Math.abs(robotAngleError) < TOLERANCE) {
            startNextCommand();
        }
    }

    /**
     * Aims the robot at the target using a navigation image.
     */
    public void aim() { //TODO Clean the camera lenses and make sure they are in focus.
        if(onFirstLoop) {

            targetVisible2 = false;

            for (int i = 0; i < 20; i++) { //TODO Tweak the number of times this loops for optimum efficiency
                targetVisible = false;

                for (VuforiaTrackable trackable : allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        targetVisible = true;
                        targetVisible2 = targetVisible;

                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocation = robotLocationTransform;
                        }

                        break;
                    }
                }
                if (targetVisible) {
                    robotPosition = lastLocation.getTranslation();
                    robotRotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, RADIANS);
                    targetAngle += currentCommand.power * Math.atan((robotPosition.get(2) - currentCommand.offset) / 1828.8);
                    if (robotPosition.get(2) > highest) {
                        highest = robotPosition.get(2);
                    }
                    if (robotPosition.get(2) < lowest) {
                        lowest = robotPosition.get(2);
                    }
                }
            }
            onFirstLoop = false;
            targetAngle /= 20;
            robotAngleError = targetAngle - currentRobotAngle;
        }
        if(Math.abs(robotAngleError) < TOLERANCE) {
            startNextCommand();
        }
    }

    /**
     * Changes the elevator position.
     */
    public void elevator() {
        elevator.setPosition(currentCommand.elevatorPosition.positionValue);
        startNextCommand();
    }

    /**
     * Toggles the shooter motor wheel.
     */
    public void shooterRev() {
        shooter.setPower(currentCommand.power * -1);
        startNextCommand();
    }

    /**
     * Waits for a time. Could not use wait() because it is a final used elsewhere.
     */
    public void pause() {
        if(time > currentCommand.duration) {
            startNextCommand();
        }
    }

    /**
     * Triggers the kicker servo.
     */
    public void kicker() {
        if(time < 1) {
            kicker.setPosition(0.8);
        } else {
            kicker.setPosition(1.0);
            startNextCommand();
        }
    }

    /**
     * Starts the next command in the sequence.
     */
    public void startNextCommand() {
        commandIndex ++;
        if (commandIndex < commands.size()) {
            currentCommand = commands.get(commandIndex);
            onFirstLoop = true;
            resetStartTime();

        }
    }

    public abstract ArrayList<Command> getCommands();

}