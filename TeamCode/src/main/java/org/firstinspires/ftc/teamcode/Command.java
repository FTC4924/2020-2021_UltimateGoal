package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.*;
/**
 * Created by Brendan Clark on 09/24/2020 at 11:54 AM.
 */

public class Command {

    public double moveSpeed = 0;
    public double moveAngle = 0;
    public double moveDuration = 0;
    public CommandType commandType;

    public Command(CommandType commandType, double angle, double speed, double duration) {

        this.commandType = commandType;

        if (commandType == CommandType.MOVE) {
            moveSpeed = speed;
            moveAngle = angle - Math.PI/2;
            moveDuration = duration;
        }

    }
}
