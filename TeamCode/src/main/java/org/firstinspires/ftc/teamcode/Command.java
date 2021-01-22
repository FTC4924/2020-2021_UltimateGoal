package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.*;
/**
 * Created by Brendan Clark on 09/24/2020 at 11:54 AM.
 */

public class Command {

    public CommandType commandType;
    public double duration = 0;
    public double angle = 0;
    public double speed = 0;

    public Command(CommandType commandType, double duration, double angle, double speed) {

        this.commandType = commandType;
        this.duration = duration;
        this.angle = Math.toRadians(angle) - Math.PI/2;
        this.speed = speed;

    }

    public Command(CommandType commandType, double angle) {

        this.commandType = commandType;
        this.angle = Math.toRadians(angle);

    }

}
