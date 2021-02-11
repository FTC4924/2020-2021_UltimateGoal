package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Constants.*;


/**
 * Created by Brendan Clark on 09/24/2020 at 12:01 PM.
 */

@Autonomous(name = "BluePowerShots")
public class BluePowerShots extends AutoBaseEncoder {
    protected ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(

                        new Command(CommandType.MOVE, 5.7, 0.0, .8),
                        new Command(CommandType.SHOOTER_REV),
                        new Command(CommandType.DETECT_IMAGE), //TODO Finish the autonomous code using the encoders and duplicate for the blue side.
                        new Command(CommandType.TURN, AngleUnit.RADIANS, getAimAngle(allianceColor.BLUE, 1275.0)),
                        new Command(CommandType.ELEVATOR, ElevatorPositions.RING_ONE),
                        new Command(CommandType.KICKER),
                        new Command(CommandType.TURN, AngleUnit.RADIANS, getAimAngle(allianceColor.BLUE, 1470.0)),
                        new Command(CommandType.ELEVATOR, ElevatorPositions.RING_TWO),
                        new Command(CommandType.KICKER),
                        new Command(CommandType.TURN, AngleUnit.RADIANS, getAimAngle(allianceColor.BLUE, 1685.0)),
                        new Command(CommandType.ELEVATOR, ElevatorPositions.RING_THREE),
                        new Command(CommandType.KICKER),
                        new Command(CommandType.SHOOTER_REV),
                        new Command(CommandType.TURN, AngleUnit.DEGREES, 0.0),
                        new Command(CommandType.MOVE, 1.1, 0.0, 1.0)
                        //TODO Add a turn to 0 at the end
                        //TODO "If we get the elevator junk working the shooter junk is consistently shooting well"-Brendan, 2021

                )
        );
    }
}