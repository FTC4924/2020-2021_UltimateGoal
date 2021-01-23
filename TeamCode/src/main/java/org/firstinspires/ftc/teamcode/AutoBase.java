package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.teamcode.Constants.TOLERANCE;
import static org.firstinspires.ftc.teamcode.Command.*;


/**
 * Created by Brendan Clark on 09/24/2020 at 11:51 AM.
 */

public abstract class AutoBase extends OpMode {

    boolean angleSet;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private BNO055IMU imu;

    private Orientation angles;
    private double currentRobotAngle;

    private double targetAngle;
    private double robotAngleError;

    ArrayList<Command> commands;
    org.firstinspires.ftc.teamcode.Command currentCommand;
    int commandIndex;

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

        angles = null;
        currentRobotAngle = 0.0;
        targetAngle = 0;

        commands = getCommands();
        currentCommand = commands.get(0);
        commandIndex = 0;

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
                    leftFrontPower *= currentCommand.speed;
                    leftBackPower *= currentCommand.speed;
                    rightFrontPower *= currentCommand.speed;
                    rightBackPower *= currentCommand.speed;
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
        }

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
