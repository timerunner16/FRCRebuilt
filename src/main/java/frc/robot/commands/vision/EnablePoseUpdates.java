package frc.robot.commands.vision;

import frc.robot.subsystems.Vision;
import frc.robot.testingdashboard.Command;

public class EnablePoseUpdates extends Command {
    public EnablePoseUpdates() {
        super(Vision.getInstance(), "VisionProperties", "EnablePoseUpdates");
    }

    @Override
    public void initialize() {
        Vision.getInstance().enablePoseUpdates();
    }

    @Override
    public boolean isFinished() {return true;}
}
