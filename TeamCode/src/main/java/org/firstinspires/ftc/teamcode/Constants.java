package org.firstinspires.ftc.teamcode;

public class Constants {

    protected static final double TOLERANCE = 0.05;
    protected static final double TURNING_REDUCTION = 1.0;
    protected static final double BRISTLES_POWER = 1.0;
    protected static final double SHOOTER_POWER = 1.0;
    protected static final double FUNNEL_LEFT_DOWN = 0.0;
    protected static final double FUNNEL_RIGHT_DOWN = 0.0;
    protected static final double FUNNEL_LEFT_UP = 0.0;
    protected static final double FUNNEL_RIGHT_UP = 0.0;
    protected static final double SHOOTER_DEFAULT_POSITION = 0.0;
    protected static final double SHOOTER_MANUAL_REDUCTION = 100;
    protected static final double KICKER_REDUCTION = 0.3;

    private static final String VUFORIA_KEY = "AaeF/Hb/////AAABmXyUA/dvl08Hn6O8IUco1axEjiRtYCVASeXGzCnFiMaizR1b3cvD+SXpU1UHHbSpnyem0dMfGb6wce32IWKttH90xMTnLjY4aXBEYscpQbX/FzUi6uf5M+sXDVNMtaVxLDGOb1phJ8tg9/Udb1cxIUCifI+AHmcwj3eknyY1ZapF81n/R0mVSmuyApS2oGQLnETWaWK+kxkx8cGnQ0Nj7a79gStXqm97obOdzptw7PdDNqOfSLVcyKCegEO0zbGoInhRMDm0MPPTxwnBihZsjDuz+I5kDEZJZfBWZ9O1PZMeFmhe6O8oFwE07nFVoclw7j2P6qHbsKTabg3w9w4ZdeTSZI4sV2t9OhbF13e0MWeV";

    protected enum ElevatorPositions {

        DOWN(0.7),
        MIDDLE(0.385), //was 0.45 on 12/23/2020, elevator was too low
        RING_ONE(0.36), //was 0.365 on 12/23/2020, elevator was too low
        RING_TWO(0.33), //was 0.345 on 12/23/2020, elevator was too low
        RING_THREE(0.31); //was 0.325 on 12/23/2020, elevator was too low

        public final double positionValue;

        ElevatorPositions(double positionValue) {

            this.positionValue = positionValue;

        }
    }
}
