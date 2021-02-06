package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Constants.*;


/**
 * Created by Brendan Clark on 09/24/2020 at 12:01 PM.
 */

@Autonomous(name = "RedHighGoalEncoder")
public class RedHighGoalEncoder extends AutoBaseEncoder {

    public ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(

                        new Command(CommandType.MOVE, 5.5, 0.0, 1.0),
                        new Command(CommandType.SHOOTER_REV, 0.68),
                        new Command(CommandType.AIM, 4.0, 300.0, -1) //TODO Finish the autonomous code using the encoders and duplicate for the blue side.


                )
        );
    }

}