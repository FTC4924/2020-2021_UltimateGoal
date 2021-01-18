package org.firstinspires.ftc.teamcode;

/**
 * Contains constants for all of the programs in one file for easy access.
 */
public class Constants {

    public enum CommandType {

        MOVE

    }

    private static final String VUFORIA_KEY = "AaeF/Hb/////AAABmXyUA/dvl08Hn6O8IUco1axEjiRtYCVASeXGzCnFiMaizR1b3cvD+SXpU1UHHbSpnyem0dMfGb6wce32IWKttH90xMTnLjY4aXBEYscpQbX/FzUi6uf5M+sXDVNMtaVxLDGOb1phJ8tg9/Udb1cxIUCifI+AHmcwj3eknyY1ZapF81n/R0mVSmuyApS2oGQLnETWaWK+kxkx8cGnQ0Nj7a79gStXqm97obOdzptw7PdDNqOfSLVcyKCegEO0zbGoInhRMDm0MPPTxwnBihZsjDuz+I5kDEZJZfBWZ9O1PZMeFmhe6O8oFwE07nFVoclw7j2P6qHbsKTabg3w9w4ZdeTSZI4sV2t9OhbF13e0MWeV";

    protected static final double TOLERANCE = 0.05;

    protected static final double TURNING_REDUCTION = 1.0;

    protected static final double FUNNEL_LEFT_DOWN = 0.1;
    protected static final double FUNNEL_RIGHT_DOWN = 0.1;
    protected static final double FUNNEL_LEFT_UP = 0.0;
    protected static final double FUNNEL_RIGHT_UP = 0.0;

    protected static final double BRISTLES_DEFAULT_POWER = 0.6;

        protected enum ElevatorPositions {
            // Coach Ethan wanted these changed (and the way the servo is attached) so that way DOWN = 1 so we don't force anything like we do
            // was 0.7, 0.385, 0.36, 0.33, and 0.31
            DOWN(1.0),
            MIDDLE(0.62),
            RING_ONE(0.595),
            RING_TWO(0.57),
            RING_THREE(0.55);

            public final double positionValue;

            ElevatorPositions(double positionValue) {

                this.positionValue = positionValue;

            }

        }

    protected static final double KICKER_REDUCTION = 0.3;

    protected static final double SHOOTER_LIFTER_MAX_POSITION = 0.8;
    protected static final double SHOOTER_LIFTER_MIN_POSITION = 0.2;
    protected static final double SHOOTER_LIFTER_DEFAULT_POSITION = 0.5;
    protected static final double SHOOTER_LIFTER_REDUCTION = .005;

    protected static final double SHOOTER_POWER = .75;

}
