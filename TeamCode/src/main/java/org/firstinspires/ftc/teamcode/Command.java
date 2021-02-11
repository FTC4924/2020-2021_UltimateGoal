package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import static org.firstinspires.ftc.teamcode.Constants.*;
/**
 * Created by Brendan Clark on 09/24/2020 at 11:54 AM.
 */

public class Command {

    public CommandType commandType;
    public double duration = 0;
    public double distance = 0;
    public double angle = 0;
    public double power = 0;

    public ElevatorPositions elevatorPosition;

    public Command(CommandType commandType, double distance, double angle, double power) {

        this.commandType = commandType;
        this.angle = Math.toRadians(angle) ;
        this.power = power;
        this.distance = distance;

    }

    public Command(CommandType commandType, AngleUnit angleUnit, double angle) {

        this.commandType = commandType;
        if(angleUnit.bVal == 0) {
            this.angle = Math.toRadians(angle);
        } else {
            this.angle = angle;
        }
    }

    public Command(CommandType commandType, double duration) {

    }

    public Command(CommandType commandType, ElevatorPositions elevatorPosition) {
        this.commandType = commandType;
        this.elevatorPosition = elevatorPosition;
    }

    public Command(CommandType commandType) {
        this.commandType = commandType;
    }



}
