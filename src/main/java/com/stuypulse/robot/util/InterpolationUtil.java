package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class InterpolationUtil {
    public static InterpolatingDoubleTreeMap interpolator;
    public static double[][] interpolationValues;
 
    static {
        interpolator = new InterpolatingDoubleTreeMap();  
        interpolationValues = Settings.Superstructure.interpolation.interpolationValues;
        for (double[] interpolationValue : interpolationValues) {
            interpolator.put(interpolationValue[0], interpolationValue[1]);
        }
    }

    public static double getRpmfromdistance(double distanceMeters) {
        return interpolator.get(distanceMeters);
    }
    
}
