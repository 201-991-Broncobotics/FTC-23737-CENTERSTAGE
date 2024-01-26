package org.firstinspires.ftc.teamcode.Roadrunner.drive.advanced;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Simple static field serving as a storage medium for the bot's pose.
 * This allows different classes/opmodes to set and read from a central source of truth.
 * A static field allows data to persist between opmodes.
 */
public class OpVariableStorage {
    public static Pose2d currentPose = new Pose2d(0, 0, Math.toRadians(90));

    public static double rotationChange = 0.5;

    //public static double VFBPosition = 0;

}