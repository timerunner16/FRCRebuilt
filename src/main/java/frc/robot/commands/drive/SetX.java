package frc.robot.commands.drive;

import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.Command;

public class SetX extends Command {
    private final Drive m_drive;
    public SetX() {
        super(Drive.getInstance(), "Drive", "SetX");
        m_drive = Drive.getInstance();

    }

    @Override
    public void execute() {
        m_drive.setX();
    }
}
