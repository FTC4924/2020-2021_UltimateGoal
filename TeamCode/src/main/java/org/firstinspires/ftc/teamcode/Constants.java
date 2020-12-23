package org.firstinspires.ftc.teamcode;

public class Constants {

    protected static final double TOLERANCE = 0.05;
    protected static final double TURNING_REDUCTION = 1.0;
    protected static final double BRISTLES_POWER = 1.0;
    protected static final double SHOOTER_POWER = 1.0;

    protected enum ElevatorPositions {

        DOWN(0.7),
        MIDDLE(0.45),
        RING_ONE(0.365),
        RING_TWO(0.345),
        RING_THREE(0.325);

        public final double positionValue;

        private ElevatorPositions(double positionValue) {

            this.positionValue = positionValue;

        }
    }
}
