
package com.stuypulse.robot.constants;

import java.util.function.Supplier;

import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/** This interface stores information about each camera. */
public interface Cameras {

    public Camera[] LimelightCameras = new Camera[] {
        //6.5, -11.3, 7.9
        new Camera("limelight", new Pose3d(Units.inchesToMeters(-11.308),Units.inchesToMeters(6.5), Units.inchesToMeters(7.9), new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(35.6), Units.degreesToRadians(180))), () -> Settings.EnabledSubsystems.LIMELIGHT.get() ),
    };

    public static class Camera {
        private String name;
        private Pose3d location;
        private Supplier<Boolean> isEnabled;

        public Camera(String name, Pose3d location, Supplier<Boolean> isEnabled) {
            this.name = name;
            this.location = location;
            this.isEnabled = isEnabled;
        }

        public String getName() {
            return name;
        }

        public Pose3d getLocation() {
            return location;
        }

        public boolean isEnabled() {
            return isEnabled.get();
        }
    }
}