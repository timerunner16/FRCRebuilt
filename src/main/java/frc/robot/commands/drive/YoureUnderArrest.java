// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import frc.robot.testingdashboard.Command;

import java.io.Console;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class YoureUnderArrest extends Command {
  /** Creates a new YoureUnderArrest. */
  Drive m_drive;
  ChassisSpeeds m_Speeds;

  public YoureUnderArrest() {
    super(Drive.getInstance(), "Auto Commands", "YoureUnderArrest");
    m_drive = Drive.getInstance();
    m_Speeds = new ChassisSpeeds(0, 0, 0);
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(m_Speeds);
    System.out.print("hallo");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.print("baiii");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
