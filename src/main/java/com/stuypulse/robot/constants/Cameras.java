
package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/** This interface stores information about each camera. */
public interface Cameras {
    
    public Camera[] LimelightCameras = new Camera[] {
        //TODO: Set up camera pose
        new Camera("limelight-turret", new Pose3d(Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))), Settings.EnabledSubsystems.LIMELIGHT_TURRET)
    };

    public static class Camera {
        private String name;
        private Pose3d location;
        private SmartBoolean isEnabled;

        public Camera(String name, Pose3d location, SmartBoolean isEnabled) {
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

        public void setEnabled(boolean enabled) {
            this.isEnabled.set(enabled);
        }
    }
}