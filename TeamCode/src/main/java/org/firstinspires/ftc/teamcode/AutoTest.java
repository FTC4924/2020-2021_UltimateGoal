package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Constants.*;


/**
 * Created by Brendan Clark on 09/24/2020 at 12:01 PM.
 */

@Autonomous(name = "AutoTest")
public class AutoTest extends AutoBase {
    protected AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }
    public ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(
                        new Command(CommandType.DETECT_RING_NUMBER,
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Command(CommandType.MOVE, AngleUnit.DEGREES, 3, -90.0, .8)
                                        )
                                ),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Command(CommandType.MOVE, AngleUnit.DEGREES, 5, -90.0, .8)
                                        )
                                ),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Command(CommandType.MOVE, AngleUnit.DEGREES, 7, -90.0, .8),
                                                new Command(CommandType.WAIT, 5),
                                                new Command(CommandType.DETECT_RING_NUMBER,
                                                        new ArrayList<>(
                                                                Arrays.asList(
                                                                        new Command(CommandType.MOVE,  AngleUnit.DEGREES, 5, -90.0, .8)
                                                                )
                                                        ),
                                                        new ArrayList<>(
                                                                Arrays.asList(
                                                                        new Command(CommandType.SHOOTER_REV),
                                                                        new Command(CommandType.SHOOTER_REV)
                                                                )
                                                        ),
                                                        new ArrayList<>(
                                                                Arrays.asList(
                                                                        new Command(CommandType.TURN, AngleUnit.DEGREES, 90)
                                                                )
                                                        )
                                                )
                                        )
                                )
                        ),
                        new Command(CommandType.TURN, AngleUnit.DEGREES, -90)
                )
        );
    }

}