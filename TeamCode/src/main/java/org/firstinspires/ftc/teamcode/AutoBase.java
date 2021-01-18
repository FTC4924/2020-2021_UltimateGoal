package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.teamcode.Constants.TOLERANCE;
import static org.firstinspires.ftc.teamcode.Command.*;


/**
 * Created by Brendan Clark on 09/24/2020 at 11:51 AM.
 */

public abstract class AutoBase extends OpMode {
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

        commands = getCommands();
        currentCommand = commands.get(0);
        commandIndex = 0;

        resetStartTime();

    }

    public void start() {
        resetStartTime();
    }

    public void loop() {
        telemetry.addData("time", time);

        double leftFrontPower = 0;
        double leftBackPower = 0;
        double rightFrontPower = 0;
        double rightBackPower = 0;
        robotAngleError = currentRobotAngle - targetAngle;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, RADIANS);
        currentRobotAngle = angles.firstAngle;

        switch (currentCommand.commandType) {

            case MOVE:

                /*Holonomic controls according to what direction the robot is facing when we start the
                program*/
                if (time < currentCommand.moveDuration) {

                    /*Determines what power each wheel should get based on the angle we get from the stick
                    plus the current robot angle so that the controls are independent of what direction the
                    robot is facing*/
                    leftFrontPower = Math.cos(currentCommand.moveAngle + Math.PI / 4 + currentRobotAngle) * -1;
                    leftBackPower = Math.sin(currentCommand.moveAngle + Math.PI / 4 + currentRobotAngle);
                    rightFrontPower = Math.sin(currentCommand.moveAngle + Math.PI / 4 + currentRobotAngle) * -1;
                    rightBackPower = Math.cos(currentCommand.moveAngle + Math.PI / 4 + currentRobotAngle);

                    // Adjusts motor speed.
                    leftFrontPower *= currentCommand.moveSpeed;
                    leftBackPower *= currentCommand.moveSpeed;
                    rightFrontPower *= currentCommand.moveSpeed;
                    rightBackPower *= currentCommand.moveSpeed;
                } else {
                    startNextCommand();
                }
                break;

        }

        leftFrontPower += robotAngleError;
        leftBackPower += robotAngleError;
        rightFrontPower += robotAngleError;
        rightBackPower += robotAngleError;

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }


    public void startNextCommand() {
        commandIndex ++;
        if (commandIndex < commands.size()) {
            currentCommand = commands.get(commandIndex);

            resetStartTime();

        }
    }

    public abstract ArrayList<Command> getCommands();

}
