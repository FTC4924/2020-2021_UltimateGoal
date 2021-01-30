package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.*;
/**
 * Created by Brendan Clark on 09/24/2020 at 11:54 AM.
 */

public class Command {

    public CommandType commandType;
    public double duration = 0;
    public double angle = 0;
    public double power = 0;
    public double offset = 0;
    public ElevatorPositions elevatorPosition;

    public Command(CommandType commandType, double duration, double angle, double power) {

        this.commandType = commandType;
        this.duration = duration;
        this.angle = Math.toRadians(angle) ;
        this.power = power;

    }

    public Command(CommandType commandType, double angle) {

        this.commandType = commandType;
        this.angle = Math.toRadians(angle);
        this.duration = angle;
        this.power = angle;

    }

    public Command(CommandType commandType, double duration, double offset, int power) {
        this.commandType = commandType;
        this.duration = duration;
        this.offset = offset;
        this.power = power;
    }

    public Command(CommandType commandType, ElevatorPositions elevatorPosition) {
        this.commandType = commandType;
        this.elevatorPosition = elevatorPosition;
    }

    public Command(CommandType commandType) {
        this.commandType = commandType;
    }



}
