package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;

public class TrajectorySolver {
    public static class TrajectoryParameters {
        public double velocity;
        public double theta_yaw;
        public double theta_pitch;
    }

    public static class TrajectoryConditions {
        // required
        public Pose3d start;
        public Pose3d target;

        public double time = 1;

        // control variables, only one needed
        public double theta;
        public double velocity;
    }

    public enum SolveType {
        CONTROL_THETA,
        CONTROL_VELOCITY
    }

    /**
     * Calculate the launch parameters of a trajectory that will hit the target.
     * 
     * @param conditions Conditions that the trajectory must meet.
     * @param type Which control variable to use from conditions, theta or velocity.
     * while firing from the specified conditions.
     */
    public static TrajectoryParameters solveTrajectory(TrajectoryConditions conditions, SolveType type) {
        double dx = conditions.start.toPose2d().getTranslation()
                .getDistance(conditions.target.toPose2d().getTranslation());
        double dy = conditions.target.getZ() - conditions.start.getZ();

        double theta_yaw = conditions.start.toPose2d().getTranslation()
                .minus(conditions.target.toPose2d().getTranslation())
                .getAngle().getRadians();

        double vx = dx/conditions.time;

        switch (type) {
            case CONTROL_THETA: {
                // dy = vyt + 1/2at2
                // vy = dy/t - 1/2at
                double vy = (dy/conditions.time - (0.5 * -9.8 * conditions.time));
                double velocity = Math.sqrt(vx*vx+vy*vy);

                TrajectoryParameters parameters = new TrajectoryParameters();
                parameters.velocity = velocity;
                parameters.theta_yaw = theta_yaw;
                parameters.theta_pitch = conditions.theta;

                return parameters;
            } case CONTROL_VELOCITY: {
                // vx = vsin(theta)
                // theta = asin(vx/v)
                double theta_pitch = Math.asin(vx/conditions.velocity);

                TrajectoryParameters parameters = new TrajectoryParameters();
                parameters.velocity = conditions.velocity;
                parameters.theta_yaw = theta_yaw;
                parameters.theta_pitch = theta_pitch;
                return parameters;
            } default: return null;
        }
    }    
}
