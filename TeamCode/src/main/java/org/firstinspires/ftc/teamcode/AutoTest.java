package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Constants.*;


/**
 * Created by Brendan Clark on 09/24/2020 at 12:01 PM.
 */

@Autonomous(name = "AutoTest")
public class AutoTest extends AutoBase {

    public ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(

                        new Command(CommandType.MOVE, 2, 0, 1.0),
                        new Command(CommandType.SHOOTER_REV, SHOOTER_POWER),
                        new Command(CommandType.AIM, 2),
                        new Command(CommandType.WAIT, 2),
                        new Command(CommandType.ELEVATOR, ElevatorPositions.RING_ONE),
                        new Command(CommandType.WAIT, 2),
                        new Command(CommandType.ELEVATOR, ElevatorPositions.RING_TWO),
                        new Command(CommandType.WAIT, 2),
                        new Command(CommandType.ELEVATOR, ElevatorPositions.RING_THREE),
                        new Command(CommandType.WAIT, 2),
                        new Command(CommandType.SHOOTER_REV, 0.0)

                )
        );
    }

}