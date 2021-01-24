package org.firstinspires.ftc.teamcode;

/**
 * Contains constants for all of the programs in one file for easy access.
 */
public class Constants {

    protected static final String VUFORIA_KEY = "AaeF/Hb/////AAABmXyUA/dvl08Hn6O8IUco1axEjiRtYCVASeXGzCnFiMaizR1b3cvD+SXpU1UHHbSpnyem0dMfGb6wce32IWKttH90xMTnLjY4aXBEYscpQbX/FzUi6uf5M+sXDVNMtaVxLDGOb1phJ8tg9/Udb1cxIUCifI+AHmcwj3eknyY1ZapF81n/R0mVSmuyApS2oGQLnETWaWK+kxkx8cGnQ0Nj7a79gStXqm97obOdzptw7PdDNqOfSLVcyKCegEO0zbGoInhRMDm0MPPTxwnBihZsjDuz+I5kDEZJZfBWZ9O1PZMeFmhe6O8oFwE07nFVoclw7j2P6qHbsKTabg3w9w4ZdeTSZI4sV2t9OhbF13e0MWeV";
    protected static final float MM_Per_Inch = 25.4f;
    protected static final float CAMERA_FORWARD_DISPLACEMENT = 57;
    protected static final float CAMERA_VERTICAL_DISPLACEMENT = 0;
    protected static final float CAMERA_LEFT_DISPLACEMENT = 184;
    protected static final float CAMERA_X_ROTATE = 0;
    protected static final float CAMERA_Y_ROTATE = 0;
    protected static final float CAMERA_Z_ROTATE = 0;

    public enum CommandType {

        MOVE,
        TURN

    }

    protected static final double TOLERANCE = 0.05;

    protected static final double TURNING_REDUCTION = 1.0;

    protected static final double FUNNEL_LEFT_DOWN = 0.3;
    protected static final double FUNNEL_RIGHT_DOWN = 0.3;
    protected static final double FUNNEL_LEFT_UP = 0.0;
    protected static final double FUNNEL_RIGHT_UP = 0.0;

    protected static final double BRISTLES_DEFAULT_POWER = 0.6;

    protected enum ElevatorPositions {

        DOWN(1.0),
        MIDDLE(0.62),
        RING_ONE(0.595),
        RING_TWO(0.57),
        RING_THREE(0.55),
        UNJAM(0.52); //Requested by Serena, added by Coach Ethan 1/19/2021

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

    protected static final double SHOOTER_POWER = .70;

}
