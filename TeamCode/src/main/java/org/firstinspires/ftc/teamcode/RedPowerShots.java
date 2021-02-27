package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Constants.*;


/**
 * Created by Brendan Clark on 09/24/2020 at 12:01 PM.
 */

@Autonomous(name = "RedPowerShots")
public class RedPowerShots extends AutoBase {
    protected AllianceColor getAllianceColor() {
        return AllianceColor.RED;
    }
    public ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(
                        new Command(CommandType.DETECT_RING_NUMBER,
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Command(CommandType.TURN, AngleUnit.DEGREES, 86.5),
                                                new Command(CommandType.MOVE, AngleUnit.DEGREES, 6.6, -3.5, 1),
                                                new Command(CommandType.MOVE, AngleUnit.DEGREES, 0.9, -185.0, 1),
                                                new Command(CommandType.TURN, AngleUnit.DEGREES, 0.0),
                                                new Command(CommandType.SHOOTER_REV),
                                                new Command(CommandType.DETECT_IMAGE), //TODO Finish the autonomous code using the encoders and duplicate for the blue side.
                                                new Command(CommandType.TURN, AngleUnit.RADIANS, getAimAngle(500.0)),
                                                new Command(CommandType.ELEVATOR, ElevatorPositions.RING_ONE),
                                                new Command(CommandType.KICKER),
                                                new Command(CommandType.TURN, AngleUnit.RADIANS, getAimAngle(650.0)),
                                                new Command(CommandType.ELEVATOR, ElevatorPositions.RING_TWO),
                                                new Command(CommandType.KICKER),
                                                new Command(CommandType.TURN, AngleUnit.RADIANS, getAimAngle(800.0)),
                                                new Command(CommandType.ELEVATOR, ElevatorPositions.RING_THREE),
                                                new Command(CommandType.KICKER),
                                                new Command(CommandType.SHOOTER_REV),
                                                new Command(CommandType.ELEVATOR, ElevatorPositions.DOWN),
                                                new Command(CommandType.MOVE, AngleUnit.DEGREES, 2.5, 180.0, 1.0),
                                                new Command(CommandType.MOVE, AngleUnit.DEGREES, 5.0, 90, 1.0),
                                                new Command(CommandType.MOVE, AngleUnit.DEGREES, 3.4, 0, 1.0),
                                                new Command(CommandType.TURN, AngleUnit.DEGREES, 0.0)
                                        )
                                ),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Command(CommandType.TURN, AngleUnit.DEGREES, 100.0),
                                                new Command(CommandType.MOVE, AngleUnit.DEGREES, 8.8, 10.0, 1),
                                                new Command(CommandType.MOVE, AngleUnit.DEGREES, 3.6, 210.0, 1),
                                                new Command(CommandType.TURN, AngleUnit.DEGREES, 0.0),
                                                new Command(CommandType.SHOOTER_REV),
                                                new Command(CommandType.DETECT_IMAGE), //TODO Finish the autonomous code using the encoders and duplicate for the blue side.
                                                new Command(CommandType.TURN, AngleUnit.RADIANS, getAimAngle(500.0)),
                                                new Command(CommandType.ELEVATOR, ElevatorPositions.RING_ONE),
                                                new Command(CommandType.KICKER),
                                                new Command(CommandType.TURN, AngleUnit.RADIANS, getAimAngle(650.0)),
                                                new Command(CommandType.ELEVATOR, ElevatorPositions.RING_TWO),
                                                new Command(CommandType.KICKER),
                                                new Command(CommandType.TURN, AngleUnit.RADIANS, getAimAngle(800.0)),
                                                new Command(CommandType.ELEVATOR, ElevatorPositions.RING_THREE),
                                                new Command(CommandType.KICKER),
                                                new Command(CommandType.SHOOTER_REV),
                                                new Command(CommandType.ELEVATOR, ElevatorPositions.DOWN),
                                                new Command(CommandType.MOVE, AngleUnit.DEGREES, 1.2, 0, 1.0),
                                                new Command(CommandType.TURN, AngleUnit.DEGREES, 0.0)
                                        )
                                ),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Command(CommandType.TURN, AngleUnit.DEGREES, 85.0),
                                                new Command(CommandType.MOVE, AngleUnit.DEGREES, 10.8, -2.5, 1),
                                                new Command(CommandType.MOVE, AngleUnit.DEGREES, 5.1, -182.5, 1),
                                                new Command(CommandType.TURN, AngleUnit.DEGREES, 0.0),
                                                new Command(CommandType.SHOOTER_REV),
                                                new Command(CommandType.DETECT_IMAGE), //TODO Finish the autonomous code using the encoders and duplicate for the blue side.
                                                new Command(CommandType.TURN, AngleUnit.RADIANS, getAimAngle(500.0)),
                                                new Command(CommandType.ELEVATOR, ElevatorPositions.RING_ONE),
                                                new Command(CommandType.KICKER),
                                                new Command(CommandType.TURN, AngleUnit.RADIANS, getAimAngle(650.0)),
                                                new Command(CommandType.ELEVATOR, ElevatorPositions.RING_TWO),
                                                new Command(CommandType.KICKER),
                                                new Command(CommandType.TURN, AngleUnit.RADIANS, getAimAngle(800.0)),
                                                new Command(CommandType.ELEVATOR, ElevatorPositions.RING_THREE),
                                                new Command(CommandType.KICKER),
                                                new Command(CommandType.SHOOTER_REV),
                                                new Command(CommandType.ELEVATOR, ElevatorPositions.DOWN),
                                                new Command(CommandType.MOVE, AngleUnit.DEGREES, 1.2, 0, 1.0),
                                                new Command(CommandType.TURN, AngleUnit.DEGREES, 0.0)
                                        )
                                )
                        )
                )
        );
    }
}