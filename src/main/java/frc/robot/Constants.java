package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double travelDist = (35.5 - 4)/12; //2.625 ft
    public static final double travelRot = 13;
    public static final double maxSpeed = 0.3;
    public static final double speedOut = 0.05;
    public static final double rampRate = 0.5;
    public static final boolean motorInverted = true;
    public static final double NeoTickstoFeet = travelDist/travelRot;  
    public static final double encoderTickstoFeet = (3.25 * Math.PI / 12) / 2048; //TODO
    public static final double outTolerance = 0.1;
    public static final double inTolerance = 1.875;
}
