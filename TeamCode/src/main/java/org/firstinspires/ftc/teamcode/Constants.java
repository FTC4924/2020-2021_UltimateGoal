package org.firstinspires.ftc.teamcode;

public class Constants {

    protected static final double JOYSTICK_TOLERANCE = 0.05;
    protected static final double TURNING_REDUCTION = 1;

    protected enum ElevatorPositions {

        DOWN(0),
        MIDDLE(0),
        RING_ONE(0),
        RING_TWO(0),
        RING_THREE(0);

        public final int positionValue;

        private ElevatorPositions(int positionValue) {

            this.positionValue = positionValue;

        }
    }
}
