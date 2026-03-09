// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
// import frc.robot.commands.Lights.BlinkLights;
import frc.robot.subsystems.Drive;
import frc.robot.testingdashboard.Command;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.utils.drive.TargetPose;

public class PathFindToPose extends Command {
  private Drive m_drive;
  private edu.wpi.first.wpilibj2.command.Command m_currentPathCommand;
  private Supplier<TargetPose> m_targetSupplier;
  private TargetPose m_currentTarget;

  TDNumber TDCurrentTargetX;
  TDNumber TDCurrentTargetY;
  TDNumber TDCurrentTargetAngle;

  // BlinkLights m_blinkLights;

  /** Creates a new TargetDrive. */
  public PathFindToPose(Supplier<TargetPose> targetSupplier) {
    super(Drive.getInstance(), "Drive", "DriveToPose");
    m_drive = Drive.getInstance();
    m_targetSupplier = targetSupplier;
    addRequirements(m_drive);

//     m_blinkLights = new BlinkLights(Constants.Color.red);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_currentTarget = m_targetSupplier.get();

    TDCurrentTargetX = new TDNumber(m_drive, "Test Outputs", "Current Target X");
    TDCurrentTargetX.set(m_currentTarget.getPose().getX());
    TDCurrentTargetY = new TDNumber(m_drive, "Test Outputs", "Current Target Y");
    TDCurrentTargetY.set(m_currentTarget.getPose().getY());
    TDCurrentTargetAngle = new TDNumber(m_drive, "Test Outputs", "Current Target Angle");
    TDCurrentTargetAngle.set(m_currentTarget.getPose().getRotation().getDegrees());
//    m_blink.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_currentPathCommand == null || m_currentTarget == null)
    {
      Pose2d target = m_currentTarget.getReversedApproach()? 
      new Pose2d(m_currentTarget.getPose().getX(), m_currentTarget.getPose().getY(), m_currentTarget.getPose().getRotation().plus(Rotation2d.k180deg)) 
      : m_currentTarget.getPose();
      m_currentPathCommand = AutoBuilder.pathfindToPose(target,
                                                        new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared, 
                                                        Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
                                                        Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared),
                                                        0);
      m_currentPathCommand.initialize();
    }
    m_currentPathCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
//    m_blinkLights.cancel();
    
    if (m_currentPathCommand != null) {
      m_currentPathCommand.end(interrupted);
      m_currentPathCommand = null;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_currentPathCommand != null) {
      return m_currentPathCommand.isFinished();
    }
    return true;
  }
}