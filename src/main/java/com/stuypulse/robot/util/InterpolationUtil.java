package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class InterpolationUtil {
    public static InterpolatingDoubleTreeMap interpoator;
    public static double[][] interpoationValues;
 
    static {
        interpoator = new InterpolatingDoubleTreeMap();  
        interpoationValues = Settings.Superstructure.interpolation.interpoationValues;
        for (double[] interpolationValue : interpoationValues) {
            interpoator.put(interpolationValue[0],interpolationValue[1]);
        }
    }

    public static double getRpmfromdistance(double distanceMeters) {
        return interpoator.get(distanceMeters);
    }
    
}
