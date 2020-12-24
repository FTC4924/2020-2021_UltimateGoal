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
    protected static final double SHOOTER_MANUAL_REDUCTION = 10;

    private static final String VUFORIA_KEY = "AaeF/Hb/////AAABmXyUA/dvl08Hn6O8IUco1axEjiRtYCVASeXGzCnFiMaizR1b3cvD+SXpU1UHHbSpnyem0dMfGb6wce32IWKttH90xMTnLjY4aXBEYscpQbX/FzUi6uf5M+sXDVNMtaVxLDGOb1phJ8tg9/Udb1cxIUCifI+AHmcwj3eknyY1ZapF81n/R0mVSmuyApS2oGQLnETWaWK+kxkx8cGnQ0Nj7a79gStXqm97obOdzptw7PdDNqOfSLVcyKCegEO0zbGoInhRMDm0MPPTxwnBihZsjDuz+I5kDEZJZfBWZ9O1PZMeFmhe6O8oFwE07nFVoclw7j2P6qHbsKTabg3w9w4ZdeTSZI4sV2t9OhbF13e0MWeV";

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
