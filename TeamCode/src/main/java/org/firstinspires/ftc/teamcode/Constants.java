package org.firstinspires.ftc.teamcode;

public class Constants {

    protected static final double JOYSTICK_TOLERANCE = 0.05;
    protected static final double TURNING_REDUCTION = 1;

    protected enum ElevatorPositions {

        DOWN(0.7),
        MIDDLE(0.385), //was 0.45 on 12/23/2020, elevator was too low
        RING_ONE(0.36), //was 0.365 on 12/23/2020, elevator was too low
        RING_TWO(0.33), //was 0.345 on 12/23/2020, elevator was too low
        RING_THREE(0.31); //was 0.325 on 12/23/2020, elevator was too low

        public final double positionValue;

        private ElevatorPositions(double positionValue) {

            this.positionValue = positionValue;

        }
    }
}
