// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

/**
 * Paths 5705 Team
*/
public class PathPlannerFiles {
    private static final double maxVelocity = 4.0,
                                maxAceleration = 3.0;

    PathPlannerTrajectory barrier = PathPlanner.loadPath("Barrier", new PathConstraints(maxVelocity, maxAceleration));
    PathPlannerTrajectory rectLine = PathPlanner.loadPath("RectLine", new PathConstraints(maxVelocity, maxAceleration));
    PathPlannerTrajectory courner = PathPlanner.loadPath("Courner", new PathConstraints(maxVelocity, maxAceleration));
    PathPlannerTrajectory chargue = PathPlanner.loadPath("Charge", new PathConstraints(maxVelocity, maxAceleration));

}
