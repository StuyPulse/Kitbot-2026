
// /************************ PROJECT MARY *************************/
// /* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
// /* Use of this source code is governed by an MIT-style license */
// /* that can be found in the repository LICENSE file.           */
// /***************************************************************/

// package com.stuypulse.robot.util;

// import com.stuypulse.stuylib.math.Vector2D;

// import com.stuypulse.robot.Robot;
// import com.stuypulse.robot.constants.Field;
// import com.stuypulse.robot.constants.Field.NamedTags;
// import com.stuypulse.robot.constants.Settings;
// import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;


// public interface HubUtil {

//     public enum HubTag {
//         FRONT_MID(NamedTags.BLUE_HUB_FRONT_SIDE_MID, NamedTags.RED_HUB_FRONT_SIDE_MID),
//         FRONT_LEFT(NamedTags.BLUE_HUB_FRONT_SIDE_LEFT, NamedTags.RED_HUB_FRONT_SIDE_LEFT),
//         LEFT_MID(NamedTags.BLUE_HUB_LEFT_SIDE_MID, NamedTags.RED_HUB_LEFT_SIDE_MID),
//         LEFT_RIGHT(NamedTags.BLUE_HUB_LEFT_SIDE_RIGHT, NamedTags.RED_HUB_LEFT_SIDE_RIGHT),
//         BACK_MID(NamedTags.BLUE_HUB_BACK_SIDE_MID, NamedTags.RED_HUB_BACK_SIDE_MID),
//         BACK_LEFT(NamedTags.BLUE_HUB_BACK_SIDE_LEFT, NamedTags.RED_HUB_BACK_SIDE_LEFT),
//         RIGHT_MID(NamedTags.BLUE_HUB_RIGHT_SIDE_MID, NamedTags.RED_HUB_RIGHT_SIDE_MID),
//         RIGHT_LEFT(NamedTags.BLUE_HUB_RIGHT_SIDE_LEFT, NamedTags.RED_HUB_RIGHT_SIDE_LEFT);

//         private NamedTags correspondingBlueAprilTag;
//         private NamedTags correspondingRedAprilTag;

//         private HubTag(NamedTags correspondingBlueAprilTag, NamedTags correspondingRedAprilTag) {
//             this.correspondingBlueAprilTag = correspondingBlueAprilTag;
//             this.correspondingRedAprilTag = correspondingRedAprilTag;
//         }

//         public Pose2d getCorrespondingAprilTagPose() {
//             return Robot.isBlue() ? this.correspondingBlueAprilTag.getLocation().toPose2d() : this.correspondingRedAprilTag.getLocation().toPose2d();
//         }

//         public Pose2d getBranchPoseProjectedOntoHubFace() {
//             return getCorrespondingAprilTagPose().transformBy(new Transform2d(0, Field.CENTER_OF_TROUGH_TO_BRANCH * (this.isLeftBranchRobotRelative() ? -1 : 1), Rotation2d.kZero));
//         }

//     }

//     public static HubTag getClosestHubTag() {

//         HubTag nearestTag = HubTag.FRONT_MID;
//         double closestDistance = Double.MAX_VALUE;

//         for (HubTag side : HubTag.values()) {
//             double distance = CommandSwerveDrivetrain.getInstance().getPose().minus(side.getBranchPoseProjectedOntoHubFace()).getTranslation().getNorm();
//             if (distance < closestDistance) {
//                 closestDistance = distance;
//                 nearestTag = side;
//             }
//         }

//         return nearestTag;
//     }

//     public enum HubFace {
//         FRONT(NamedTags.BLUE_HUB_FRONT_SIDE_LEFT, NamedTags.BLUE_HUB_FRONT_SIDE_MID, NamedTags.RED_HUB_FRONT_SIDE_MID, NamedTags.RED_HUB_FRONT_SIDE_LEFT, HubTag.FRONT_MID, HubTag.FRONT_LEFT),
//         LEFT(NamedTags.BLUE_HUB_LEFT_SIDE_MID, NamedTags.BLUE_HUB_LEFT_SIDE_RIGHT, NamedTags.RED_HUB_LEFT_SIDE_MID, NamedTags.RED_HUB_LEFT_SIDE_RIGHT, HubTag.LEFT_MID, HubTag.LEFT_RIGHT),
//         BACK(NamedTags.BLUE_HUB_BACK_SIDE_MID, NamedTags.BLUE_HUB_BACK_SIDE_LEFT, NamedTags.RED_HUB_BACK_SIDE_MID, NamedTags.RED_HUB_BACK_SIDE_LEFT, HubTag.BACK_MID, HubTag.BACK_LEFT),
//         RIGHT(NamedTags.BLUE_HUB_RIGHT_SIDE_MID, NamedTags.BLUE_HUB_RIGHT_SIDE_LEFT, NamedTags.RED_HUB_RIGHT_SIDE_MID, NamedTags.RED_HUB_RIGHT_SIDE_LEFT, HubTag.RIGHT_MID, HubTag.RIGHT_LEFT);

//         private NamedTags correspondingBlueAprilTagOne;
//         private NamedTags correspondingBlueAprilTagTwo;
//         private NamedTags correspondingRedAprilTagOne;
//         private NamedTags correspondingRedAprilTagTwo;
//         private HubTag hubTagMid;
//         private HubTag hubTagNotMid;

//         private HubFace(NamedTags correspondingBlueAprilTagOne, NamedTags correspondingBlueAprilTagTwo, NamedTags correspondingRedAprilTagOne, NamedTags correspondingRedAprilTagTwo, HubTag hubTagMid, HubTag hubTagNotMid) {
//             this.correspondingBlueAprilTagOne = correspondingBlueAprilTagOne;
//             this.correspondingBlueAprilTagTwo = correspondingBlueAprilTagTwo;
//             this.correspondingRedAprilTagOne = correspondingRedAprilTagOne;
//             this.correspondingRedAprilTagTwo = correspondingRedAprilTagTwo;
//             this.hubTagMid = hubTagMid;
//             this.hubTagNotMid = hubTagNotMid;
//         }

//         public HubFace rotateCCW(int faces) {
//             int index = (this.ordinal() + (faces % 6) + 6) % 6;
//             return HubFace.values()[index];
//         }

//         public HubTag getHubTagMid() {
//             return this.hubTagMid;
//         }

//         public HubTag getHubTagNotMid() {
//             return this.hubTagNotMid;
//         }

//         public HubTag getClosestHubTag() {
//             Translation2d robot = CommandSwerveDrivetrain.getInstance().getPose().getTranslation();
//             double leftBranchDistance = getHubTagMid().getBranchPoseProjectedOntoHubFace().getTranslation().getDistance(robot);
//             double rightBranchDistance = getHubTagNotMid().getBranchPoseProjectedOntoHubFace().getTranslation().getDistance(robot);
//             return leftBranchDistance < rightBranchDistance
//                 ? getHubTagMid()
//                 : getHubTagNotMid();
//         }
// drfgjgjdfkjgnfdgjnkdfgjnkdfgjnkdfjgdfjk
//         public Pose2d getCorrespondingAprilTagPose() {
//             return Robot.isBlue() ? this.correspondingBlueAprilTag.getLocation().toPose2d() : this.correspondingRedAprilTag.getLocation().toPose2d();
//         }

//         public static NamedTags getNearestReefSide() {
//             return Robot.isBlue() ? getClosestHubFace().correspondingBlueAprilTag : getClosestFace().correspondingRedAprilTag;
//         }

//         private Translation2d getLineStart() {
//             return getCorrespondingAprilTagPose().transformBy(new Transform2d(0, Field.LENGTH_OF_REEF_FACE / 2, Rotation2d.kZero)).getTranslation();
//         }

//         private Translation2d getLineEnd() {
//             return getCorrespondingAprilTagPose().transformBy(new Transform2d(0, -Field.LENGTH_OF_REEF_FACE / 2, Rotation2d.kZero)).getTranslation();
//         }

//         public Pose2d getL1ShooterTargetPose() {
//             Translation2d lineStart = getLineStart();
//             Translation2d lineEnd = getLineEnd();
//             Translation2d robot = CommandSwerveDrivetrain.getInstance().getPose().getTranslation();

//             Vector2D lineStartToEnd = new Vector2D(lineEnd.getX() - lineStart.getX(), lineEnd.getY() - lineStart.getY());
//             Vector2D lineStartToPoint = new Vector2D(robot.getX() - lineStart.getX(), robot.getY() - lineStart.getY());
            
//             double lineLengthSquared = lineStartToEnd.dot(lineStartToEnd);
//             double dotProduct = lineStartToEnd.dot(lineStartToPoint);
            
//             double t = dotProduct / lineLengthSquared; // Projection factor
//             t = Math.max(0, Math.min(1, t));
            
//             Translation2d closestPointOnReefFace = new Translation2d(lineStart.getX() + t * lineStartToEnd.x, lineStart.getY() + t * lineStartToEnd.y);

//             return new Pose2d(closestPointOnReefFace, getCorrespondingAprilTagPose().getRotation())
//                 .transformBy(new Transform2d(Constants.LENGTH_WITH_BUMPERS_METERS / 2 + Settings.Swerve.Alignment.Targets.TARGET_DISTANCE_FROM_REEF_L1_SHOOTER_FRONT, 0, Rotation2d.k180deg));
//         }

//     public static ReefFace getClosestReefFace() {
//         ReefFace nearestReefFace = ReefFace.AB;
//         double closestDistance = Double.MAX_VALUE;

//         for (ReefFace reefFace : ReefFace.values()) {
//             double distance = CommandSwerveDrivetrain.getInstance().getPose().minus(reefFace.getCorrespondingAprilTagPose()).getTranslation().getNorm();
//             if (distance < closestDistance) {
//                 closestDistance = distance;
//                 nearestReefFace = reefFace;
//             }
//         }
//         return nearestReefFace;
//     }

// }