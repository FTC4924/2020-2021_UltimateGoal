package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Constants.*;


/**
 * Created by Brendan Clark on 09/24/2020 at 12:01 PM.
 */

@Autonomous(name = "BluePowerShotsInner")
public class BluePowerShotsInner extends AutoBase {
    protected AllianceColor getAllianceColor() {
        return AllianceColor.BLUE;
    }
    protected ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(

                        new Command(CommandType.MOVE, AngleUnit.DEGREES, 5.7, 0.0, .8),
                        new Command(CommandType.SHOOTER_REV),
                        new Command(CommandType.DETECT_IMAGE), //TODO Finish the autonomous code using the encoders and duplicate for the blue side.
                        new Command(CommandType.TURN, AngleUnit.RADIANS, getAimAngle(875.0)),
                        new Command(CommandType.ELEVATOR, ElevatorPositions.RING_ONE),
                        new Command(CommandType.KICKER),
                        new Command(CommandType.TURN, AngleUnit.RADIANS, getAimAngle(1070.0)),
                        new Command(CommandType.ELEVATOR, ElevatorPositions.RING_TWO),
                        new Command(CommandType.KICKER),
                        new Command(CommandType.TURN, AngleUnit.RADIANS, getAimAngle(1285.0)),
                        new Command(CommandType.ELEVATOR, ElevatorPositions.RING_THREE),
                        new Command(CommandType.KICKER),
                        new Command(CommandType.SHOOTER_REV),
                        new Command(CommandType.ELEVATOR, ElevatorPositions.DOWN),
                        new Command(CommandType.TURN, AngleUnit.DEGREES, 0.0),
                        new Command(CommandType.MOVE, AngleUnit.DEGREES, 2.9, 0.0, 1.0),
                        new Command(CommandType.MOVE, AngleUnit.DEGREES, 2, 180.0, 1.0),
                        new Command(CommandType.MOVE, AngleUnit.DEGREES, 4.0, -90.0, 1.0)
                        //TODO Add a turn to 0 at the end
                        //TODO "If we get the elevator junk working the shooter junk is consistently shooting well"-Brendan, 2021

                )
        );
    }
}