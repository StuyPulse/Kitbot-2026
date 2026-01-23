package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.math.Vector2D;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.util.vision.AprilTag;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

import java.util.ArrayList;
import java.util.List;

/** This interface stores information about the field elements. */
public interface Field {

    public static final Field2d FIELD2D = new Field2d();

    double WIDTH = Units.inchesToMeters(317.000); 
    double LENGTH = Units.inchesToMeters(690.876);

    public static Pose3d transformToOppositeAlliance(Pose3d pose) {
        Pose3d rotated = pose.rotateBy(new Rotation3d(0, 0, Math.PI));

        return new Pose3d(
            rotated.getTranslation().plus(new Translation3d(LENGTH, WIDTH, 0)),
            rotated.getRotation());
    }

    public static Pose2d transformToOppositeAlliance(Pose2d pose) {
        Pose2d rotated = pose.rotateBy(Rotation2d.fromDegrees(180));
        return new Pose2d(
            rotated.getTranslation().plus(new Translation2d(LENGTH, WIDTH)),
            rotated.getRotation());
    }
    
    public static Translation2d transformToOppositeAlliance(Translation2d translation) {
        return new Translation2d(LENGTH - translation.getX(), WIDTH - translation.getY());
    }

    public static List<Pose2d> transformToOppositeAlliance(List<Pose2d> poses) {
        List<Pose2d> newPoses = new ArrayList<>();
        for (Pose2d pose : poses) {
            newPoses.add(transformToOppositeAlliance(pose));
        }
        return newPoses;
    }

    /*** APRILTAGS ***/

    enum NamedTags {
        RED_RIGHT_TRENCH_NZ, // #1
        RED_HUB_RIGHT_SIDE_MID,
        RED_HUB_BACK_SIDE_LEFT,
        RED_HUB_BACK_SIDE_MID,
        RED_HUB_LEFT_SIDE_MID, // #5
        RED_LEFT_TRENCH_NZ,
        RED_LEFT_TRENCH_AZ,
        RED_HUB_LEFT_SIDE_RIGHT,
        RED_HUB_FRONT_SIDE_LEFT,
        RED_HUB_FRONT_SIDE_MID, // #10
        RED_HUB_RIGHT_SIDE_LEFT,
        RED_RIGHT_TRENCH_AZ,
        RED_HP_MID,
        RED_HP_RIGHT,
        RED_TOWER_MID, // #15
        RED_TOWER_RIGHT,
        BLUE_RIGHT_TRENCH_NZ,
        BLUE_HUB_RIGHT_SIDE_MID,
        BLUE_HUB_BACK_SIDE_LEFT,
        BLUE_HUB_BACK_SIDE_MID, // #20
        BLUE_HUB_LEFT_SIDE_MID,
        BLUE_LEFT_TRENCH_NZ,
        BLUE_LEFT_TRENCH_AZ,
        BLUE_HUB_LEFT_SIDE_RIGHT,
        BLUE_HUB_FRONT_SIDE_LEFT, // #25
        BLUE_HUB_FRONT_SIDE_MID,
        BLUE_HUB_RIGHT_SIDE_LEFT,
        BLUE_RIGHT_TRENCH_AZ,
        BLUE_HP_MID,
        BLUE_HP_RIGHT, // #30
        BLUE_TOWER_MID,
        BLUE_TOWER_RIGHT; // #32

        public final AprilTag tag;

        public int getID() {
            return tag.getID();
        }

        public Pose3d getLocation() {
            return Robot.isBlue()
                ? tag.getLocation()
                : transformToOppositeAlliance(tag.getLocation());
        }

        private NamedTags() {
            tag = APRILTAGS[ordinal()];
        }
    }

    AprilTag APRILTAGS[] = {
        // 2026 Field AprilTag Layout
        new AprilTag(1,  new Pose3d(new Translation3d(Units.inchesToMeters(467.64), Units.inchesToMeters(292.31), Units.inchesToMeters(35.00)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))), 
        new AprilTag(2,  new Pose3d(new Translation3d(Units.inchesToMeters(469.11), Units.inchesToMeters(182.60), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(90)))),
        new AprilTag(3,  new Pose3d(new Translation3d(Units.inchesToMeters(445.35), Units.inchesToMeters(172.84), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(4,  new Pose3d(new Translation3d(Units.inchesToMeters(445.35), Units.inchesToMeters(158.84), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(5,  new Pose3d(new Translation3d(Units.inchesToMeters(469.11), Units.inchesToMeters(135.09), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
        new AprilTag(6,  new Pose3d(new Translation3d(Units.inchesToMeters(467.64), Units.inchesToMeters(25.37), Units.inchesToMeters(35.00)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(7,  new Pose3d(new Translation3d(Units.inchesToMeters(470.59), Units.inchesToMeters(25.37), Units.inchesToMeters(35.00)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(8,  new Pose3d(new Translation3d(Units.inchesToMeters(483.11), Units.inchesToMeters(135.09), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
        new AprilTag(9,  new Pose3d(new Translation3d(Units.inchesToMeters(492.88), Units.inchesToMeters(144.84), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(10,  new Pose3d(new Translation3d(Units.inchesToMeters(492.88), Units.inchesToMeters(158.84), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(11,  new Pose3d(new Translation3d(Units.inchesToMeters(483.11), Units.inchesToMeters(182.60), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(90)))),
        new AprilTag(12,  new Pose3d(new Translation3d(Units.inchesToMeters(470.59), Units.inchesToMeters(292.31), Units.inchesToMeters(35.00)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(13,  new Pose3d(new Translation3d(Units.inchesToMeters(650.92), Units.inchesToMeters(291.47), Units.inchesToMeters(21.75)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(14,  new Pose3d(new Translation3d(Units.inchesToMeters(650.92), Units.inchesToMeters(274.47), Units.inchesToMeters(21.75)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(15,  new Pose3d(new Translation3d(Units.inchesToMeters(650.90), Units.inchesToMeters(170.22), Units.inchesToMeters(21.75)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(16,  new Pose3d(new Translation3d(Units.inchesToMeters(650.90), Units.inchesToMeters(153.22), Units.inchesToMeters(21.75)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(17,  new Pose3d(new Translation3d(Units.inchesToMeters(183.59), Units.inchesToMeters(25.37), Units.inchesToMeters(35.00)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(18,  new Pose3d(new Translation3d(Units.inchesToMeters(182.11), Units.inchesToMeters(135.09), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
        new AprilTag(19,  new Pose3d(new Translation3d(Units.inchesToMeters(205.87), Units.inchesToMeters(144.84), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(20,  new Pose3d(new Translation3d(Units.inchesToMeters(205.87), Units.inchesToMeters(158.84), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(21,  new Pose3d(new Translation3d(Units.inchesToMeters(182.11), Units.inchesToMeters(182.60), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(90)))),
        new AprilTag(22,  new Pose3d(new Translation3d(Units.inchesToMeters(183.59), Units.inchesToMeters(292.31), Units.inchesToMeters(35.00)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(23,  new Pose3d(new Translation3d(Units.inchesToMeters(180.64), Units.inchesToMeters(292.31), Units.inchesToMeters(35.00)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(24,  new Pose3d(new Translation3d(Units.inchesToMeters(168.11), Units.inchesToMeters(182.60), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(90)))),
        new AprilTag(25,  new Pose3d(new Translation3d(Units.inchesToMeters(158.34), Units.inchesToMeters(172.84), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(26,  new Pose3d(new Translation3d(Units.inchesToMeters(158.34), Units.inchesToMeters(158.84), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(27,  new Pose3d(new Translation3d(Units.inchesToMeters(168.11), Units.inchesToMeters(135.09), Units.inchesToMeters(44.25)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(270)))),
        new AprilTag(28,  new Pose3d(new Translation3d(Units.inchesToMeters(180.64), Units.inchesToMeters(25.37), Units.inchesToMeters(35.00)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180)))),
        new AprilTag(29,  new Pose3d(new Translation3d(Units.inchesToMeters(0.30), Units.inchesToMeters(26.22), Units.inchesToMeters(21.75)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(30,  new Pose3d(new Translation3d(Units.inchesToMeters(0.30), Units.inchesToMeters(43.22), Units.inchesToMeters(21.75)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(31,  new Pose3d(new Translation3d(Units.inchesToMeters(0.32), Units.inchesToMeters(147.47), Units.inchesToMeters(21.75)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)))),
        new AprilTag(32,  new Pose3d(new Translation3d(Units.inchesToMeters(0.32), Units.inchesToMeters(164.47), Units.inchesToMeters(21.75)), new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0))))
    };

    public static boolean isValidTag(int id) {
        for (AprilTag tag : APRILTAGS) {
            if (tag.getID() == id) {
                return true;
            }
        }
        return false;
    }

    public static AprilTag getTag(int id) {
        for (AprilTag tag : APRILTAGS) {
            if (tag.getID() == id) {
                return tag;
            }
        }
        return null;
    }

    public int[] RED_HUB_TAG_IDS = {2, 3, 4, 5, 8, 9, 10, 11};
    public int[] BLUE_HUB_TAG_IDS = {18, 19, 20, 21, 24, 25, 26, 27};
    public int[] RED_TRENCH_TAG_IDS = {1, 6, 7, 12};
    public int[] BLUE_TRENCH_TAG_IDS = {17, 22, 23, 28};
    public int[] RED_TOWER_TAG_IDS = {15, 16};
    public int[] BLUE_TOWER_TAG_IDS = {31, 32};
    public int[] RED_HP_TAG_IDS = {13, 14};
    public int[] BLUE_HP_TAG_IDS = {29, 30};

}

public static Pose2d getAllianceHubPose() {
        return (Robot.isBlue() ? NamedTags.BLUE_HUB_FRONT_SIDE_MID : NamedTags.RED_HUB_FRONT_SIDE_MID)
            .getLocation().toPose2d();
    }

//     /*** HUBS ***/
//     Translation2d ALLIANCE_REEF_CENTER = new Translation2d(Units.inchesToMeters(144.0 + (93.5 - 14.0 * 2) / 2), Field.WIDTH / 2);
//     Translation2d OPPOSITE_ALLIANCE_REEF_CENTER = transformToOppositeAlliance(ALLIANCE_REEF_CENTER);
//     double LENGTH_OF_REEF_FACE = Units.inchesToMeters(37.04);
//     double CENTER_OF_REEF_TO_L1_CORNER = LENGTH_OF_REEF_FACE / 2 - Units.inchesToMeters(5);
//     double CENTER_OF_REEF_TO_REEF_FACE = Units.inchesToMeters(32.75);
//     double CENTER_OF_TROUGH_TO_BRANCH = Units.inchesToMeters(13.0/2.0);

//     /*** TRENCHES ***/

//     public static Pose2d getCatapultTargetPose(double yDistanceFromCenterline) {
//         return new Pose2d(new Translation2d(
//                 Field.LENGTH / 2 - (Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_CENTERLINE_FOR_BARGE_118), 
//                 Field.WIDTH / 2 + yDistanceFromCenterline), 
//                 Rotation2d.k180deg.plus(Settings.Swerve.Alignment.Targets.ANGLE_FROM_HORIZONTAL_FOR_118_AUTON));
//     }

//     public static Pose2d getCatapultTargetPoseAuton(double yDistanceFromCenterline) {
//         return new Pose2d(new Translation2d(
//                 Field.LENGTH / 2 - (Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_CENTERLINE_FOR_BARGE_118 + Units.inchesToMeters(5)), 
//                 Field.WIDTH / 2 + yDistanceFromCenterline), 
//                 Rotation2d.k180deg.plus(Settings.Swerve.Alignment.Targets.ANGLE_FROM_HORIZONTAL_FOR_118_AUTON));
//     }

//     /*** TOWER ***/
//     public static Pose2d getTargetPoseForProcessorShooter() {
//         return Robot.isBlue()
//             ? NamedTags.BLUE_PROCESSOR.getLocation().toPose2d().plus(new Transform2d(Constants.LENGTH_WITH_BUMPERS_METERS / 2 - 0.08, Constants.SHOOTER_Y_OFFSET, Rotation2d.k180deg))
//             : NamedTags.RED_PROCESSOR.getLocation().toPose2d().plus(new Transform2d(Constants.LENGTH_WITH_BUMPERS_METERS / 2 - 0.08, Constants.SHOOTER_Y_OFFSET, Rotation2d.k180deg));
//     }

//     public static Pose2d getTargetPoseForProcessorFroggy() {
//         return Robot.isBlue()
//             ? NamedTags.BLUE_PROCESSOR.getLocation().toPose2d().plus(new Transform2d(Constants.WIDTH_WITH_BUMPERS_METERS / 2, 0, Rotation2d.kCW_90deg))
//             : NamedTags.RED_PROCESSOR.getLocation().toPose2d().plus(new Transform2d(Constants.WIDTH_WITH_BUMPERS_METERS / 2, 0, Rotation2d.kCW_90deg));
//     }

//     /*** HPS ***/
//     public enum CoralStation {
//         BLUE_CD_CORAL_STATION(NamedTags.BLUE_CD_CORAL_STATION, new Translation2d(0, Field.WIDTH / 2 - Units.inchesToMeters(109.13)), new Translation2d(Units.inchesToMeters(65.84), 0)),
//         BLUE_KL_CORAL_STATION(NamedTags.BLUE_KL_CORAL_STATION, new Translation2d(0, Field.WIDTH / 2 + Units.inchesToMeters(109.13)), new Translation2d(Units.inchesToMeters(65.84), Field.WIDTH)),
//         RED_CD_CORAL_STATION(NamedTags.RED_CD_CORAL_STATION, new Translation2d(Field.LENGTH, Field.WIDTH / 2 + Units.inchesToMeters(109.13)), new Translation2d(Field.LENGTH - Units.inchesToMeters(65.84), Field.WIDTH)),
//         RED_KL_CORAL_STATION(NamedTags.RED_KL_CORAL_STATION, new Translation2d(Field.LENGTH, Field.WIDTH / 2 - Units.inchesToMeters(109.13)), new Translation2d(Field.LENGTH - Units.inchesToMeters(65.84), 0));
    
//         private NamedTags correspondingAprilTag;
//         private Translation2d blueOriginLineStart; // Driver station wall
//         private Translation2d blueOriginLineEnd; // Side of field

//         private CoralStation(NamedTags correspondingAprilTag, Translation2d blueOriginLineStart, Translation2d blueOriginLineEnd) {
//             this.correspondingAprilTag = correspondingAprilTag;
//             this.blueOriginLineStart = blueOriginLineStart;
//             this.blueOriginLineEnd = blueOriginLineEnd;
//         }

//         private Translation2d getLineStart() {
//             return Robot.isBlue()
//                 ? blueOriginLineStart
//                 : transformToOppositeAlliance(new Pose2d(blueOriginLineStart, Rotation2d.kZero)).getTranslation();
//         }

//         private Translation2d getLineEnd() {
//             return Robot.isBlue()
//                 ? blueOriginLineEnd
//                 : transformToOppositeAlliance(new Pose2d(blueOriginLineEnd, Rotation2d.kZero)).getTranslation();
//         }

//         private boolean isCDCoralStation() {
//             return switch (this) {
//                 case BLUE_CD_CORAL_STATION, RED_CD_CORAL_STATION -> true;
//                 default -> false;
//             };
//         }

//         // Kalimul said to add a comment that this is getting the closest point to a line
//         public Pose2d getTargetPose() {
//             Translation2d lineStart = getLineStart();
//             Translation2d lineEnd = getLineEnd();
//             Translation2d robot = CommandSwerveDrivetrain.getInstance().getPose().getTranslation();

//             Vector2D lineStartToEnd = new Vector2D(lineEnd.getX() - lineStart.getX(), lineEnd.getY() - lineStart.getY());
//             Vector2D lineStartToPoint = new Vector2D(robot.getX() - lineStart.getX(), robot.getY() - lineStart.getY());
            
//             double lineLengthSquared = lineStartToEnd.dot(lineStartToEnd);
//             double dotProduct = lineStartToEnd.dot(lineStartToPoint);
            
//             double t = dotProduct / lineLengthSquared; // Projection factor
            
//             double coralStationLength = lineStart.getDistance(lineEnd);
//             double percentToIgnoreFromDriverStationSide = (Constants.WIDTH_WITH_BUMPERS_METERS / 2 + (isCDCoralStation() ? Settings.Clearances.CLEARANCE_DISTANCE_CORAL_STATION_ALIGN_FUNNEL_SIDE : Settings.Clearances.CLEARANCE_DISTANCE_CORAL_STATION_ALIGN_FROGGY_SIDE)) / coralStationLength;
//             double percentToIgnoreFromSideWallSide = (Constants.WIDTH_WITH_BUMPERS_METERS / 2 + (isCDCoralStation() ? Settings.Clearances.CLEARANCE_DISTANCE_CORAL_STATION_ALIGN_FROGGY_SIDE : Settings.Clearances.CLEARANCE_DISTANCE_CORAL_STATION_ALIGN_FUNNEL_SIDE)) / coralStationLength;
            
//             t = Math.max(percentToIgnoreFromDriverStationSide, Math.min(1 - percentToIgnoreFromSideWallSide, t));
            
//             Translation2d closestPointOnCoralStation = new Translation2d(lineStart.getX() + t * lineStartToEnd.x, lineStart.getY() + t * lineStartToEnd.y);

//             return new Pose2d(closestPointOnCoralStation, correspondingAprilTag.getLocation().toPose2d().getRotation()).transformBy(new Transform2d(
//                 Constants.LENGTH_WITH_BUMPERS_METERS / 2 + Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_CORAL_STATION, 
//                 0, Rotation2d.kZero));
//         }

//         public Pose2d getTargetPose(boolean isLeftSideOfStation, boolean isCD) {
//             // double distance_y = (isLeftSideOfStation ? Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_CORAL_STATION_LEFT : Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_CORAL_STATION_RIGHT);
//             // double in_out = (isCD ? distance_y )
//             double distance_y;
//             if (isCD) {
//                 if (isLeftSideOfStation) {
//                     distance_y = Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_CORAL_STATION_IN + Units.inchesToMeters(6);
//                 } else {
//                     distance_y = Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_CORAL_STATION_OUT + Units.inchesToMeters(8);
//                 }
//             } else {
//                 if (!isLeftSideOfStation) {
//                     distance_y = Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_CORAL_STATION_OUT - Units.inchesToMeters(5);
//                 } else {
//                     distance_y = Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_CORAL_STATION_IN - Units.inchesToMeters(8);
//                 }
//             }
//             return correspondingAprilTag.getLocation().toPose2d().transformBy(
//                 new Transform2d(
//                     Constants.LENGTH_WITH_BUMPERS_METERS / 2 + Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_CORAL_STATION,
//                     distance_y,
//                     Rotation2d.kZero));
//         }

//         public Vector2D getHeadingAsVector() {
//             return new Vector2D(
//                 Math.cos(correspondingAprilTag.getLocation().getRotation().getZ()), 
//                 Math.sin(correspondingAprilTag.getLocation().getRotation().getZ()));
//         }

//         // https://www.youtube.com/watch?v=KHuI9bXZS74
//         public double getDistanceToStation() {
//             Vector2D A = new Vector2D(getLineStart());
//             Vector2D B = new Vector2D(getLineEnd());
//             Vector2D C = new Vector2D(CommandSwerveDrivetrain.getInstance().getPose().getTranslation());
            
//             return Math.abs((C.x - A.x) * (-B.y + A.y) + (C.y - A.y) * (B.x - A.x))
//                 / Math.sqrt(Math.pow((-B.y + A.y), 2) + Math.pow((B.x - A.x), 2));
//         }

//         public static CoralStation getClosestCoralStation() {
//             if (Robot.isBlue()) {
//                 return BLUE_CD_CORAL_STATION.getDistanceToStation() <  BLUE_KL_CORAL_STATION.getDistanceToStation()
//                     ? BLUE_CD_CORAL_STATION
//                     : BLUE_KL_CORAL_STATION;
//             }
//             else {
//                 return RED_CD_CORAL_STATION.getDistanceToStation() <  RED_KL_CORAL_STATION.getDistanceToStation()
//                     ? RED_CD_CORAL_STATION
//                     : RED_KL_CORAL_STATION;
//             }
//         }

//         public static double getDistanceToClosestStation(Pose2d pose) {
//             return getClosestCoralStation().getDistanceToStation();
//         }

//         public static CoralStation getCoralStation(boolean isCD) {
//             if (isCD) {
//                 return Robot.isBlue() ?
//                     CoralStation.BLUE_CD_CORAL_STATION : 
//                     CoralStation.RED_CD_CORAL_STATION; 
//             } else {
//                 return Robot.isBlue() ?
//                     CoralStation.BLUE_KL_CORAL_STATION :
//                     CoralStation.RED_KL_CORAL_STATION;
//             }
//         }
//     }


//     /**** EMPTY FIELD POSES ****/

//     Pose2d EMPTY_FIELD_POSE2D = new Pose2d(new Translation2d(-1, -1), new Rotation2d());
//     Pose3d EMPTY_FIELD_POSE3D = new Pose3d(-1, -1, 0, new Rotation3d());

//     public static void clearFieldObject(FieldObject2d fieldObject)  {
//         fieldObject.setPose(EMPTY_FIELD_POSE2D);
//     }
// }