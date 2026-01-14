package frc.robot.commands.vision;

import frc.robot.subsystems.Vision;
import frc.robot.testingdashboard.Command;

public class DisablePoseUpdates extends Command {
    public DisablePoseUpdates() {
        super(Vision.getInstance(), "VisionProperties", "DisablePoseUpdates");
    }

    @Override
    public void initialize() {
        Vision.getInstance().disablePoseUpdates();
    }

    @Override
    public boolean isFinished() {return true;}
}
