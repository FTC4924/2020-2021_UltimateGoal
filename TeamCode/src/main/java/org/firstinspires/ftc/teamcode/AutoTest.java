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
                        new Command(CommandType.MOVE, 1.25, 0, .5),
                        new Command(CommandType.TURN, -90),
                        new Command(CommandType.MOVE, 1.25, -90, .5),
                        new Command(CommandType.TURN, 180),
                        new Command(CommandType.MOVE, 1.25, 180, .5),
                        new Command(CommandType.TURN, 90),
                        new Command(CommandType.MOVE, 1.25, 90, .5),
                        new Command(CommandType.TURN, 0)
                )
        );
    }

}