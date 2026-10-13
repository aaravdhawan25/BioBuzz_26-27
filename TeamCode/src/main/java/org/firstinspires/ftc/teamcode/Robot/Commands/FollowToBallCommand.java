package org.firstinspires.ftc.teamcode.Robot.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.HiveVision.BallChaseFollower;
import org.firstinspires.ftc.teamcode.HiveVision.FieldPlanner;
import org.firstinspires.ftc.teamcode.Utils.Constants.FieldPlannerConstants;

public class FollowToBallCommand extends CommandBase {
    private final Follower follower;
    private final BallChaseFollower vision;
    private boolean found;

    public FollowToBallCommand(Follower follower, BallChaseFollower vision) {
        this.follower = follower;
        this.vision = vision;
    }

    @Override
    public void initialize() {
        Pose start = follower.pose();
        Pose target = FieldPlanner.bestApproach(start, vision.seeBalls(), FieldPlannerConstants.stopShort);
        Path path = target == null ? null : FieldPlanner.plan(start, target);
        found = path != null;
        if (found) follower.follow(path);
    }

    @Override
    public boolean isFinished() {
        return !found || !follower.isBusy();
    }
}