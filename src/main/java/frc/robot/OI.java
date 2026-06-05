package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.SlowSwerveDrive;
import frc.robot.commands.intake.IntakeAAAAA;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.shooter.CalibrateTurretFull;
import frc.robot.commands.shooter.FerryShoot;
import frc.robot.commands.shooter.HoodCalibrate;
import frc.robot.commands.shooter.ShootToPose;
import frc.robot.commands.spindexer.SpindexerSpin;
import frc.robot.subsystems.Drive;
import frc.robot.utils.TriggerBuilder;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.Referrable;
import frc.robot.utils.drive.SwerveDriveInputs;

public class OI {
    private static OI m_OI;

    private CommandGenericHID m_driverLeft;
    private CommandGenericHID m_driverRight;
    private CommandGenericHID m_operatorGuitar;

    private SwerveDriveInputs m_driveInputs;

    private boolean m_lastStrumSide;
    private double m_timeSinceStrum;

    private enum Submap {
        DRIVE_SYSID,
        AUTO,
        MANUAL;
    }

    private final Referrable<Submap> m_operatorSubmap = new Referrable<Submap>(Submap.AUTO);
    private final Referrable<Submap> m_driverSubmap = new Referrable<Submap>(Submap.AUTO);

    public static OI getInstance() {
        if (m_OI == null)
            m_OI = new OI();
        return m_OI;
    }

    private OI() {
        m_timeSinceStrum = Double.MAX_VALUE;
        m_lastStrumSide = false;

        m_driverLeft = new CommandXboxController(RobotMap.U_DRIVER_LEFT_CONTROLLER);
        m_driverRight = new CommandXboxController(RobotMap.U_DRIVER_RIGHT_CONTROLLER);
        m_operatorGuitar = new CommandXboxController(RobotMap.U_OPERATOR_GUITAR_CONTROLLER);

        Supplier<Double> xInput;
        Supplier<Double> yInput;
        if (RobotBase.isReal()) {
            xInput = () -> m_driverLeft.getRawAxis(0);
            yInput = () -> m_driverLeft.getRawAxis(1);
        } else {
            xInput = () -> m_driverLeft.getRawAxis(0);
            yInput = () -> -m_driverLeft.getRawAxis(1);
        }
        m_driveInputs = new SwerveDriveInputs(xInput, yInput, () -> -m_driverRight.getRawAxis(1));
    }

    public void bindControls() {
        bindRockstarLayout(m_driverLeft, m_driverRight, m_operatorGuitar);
    }

    public void bindRockstarLayout(CommandGenericHID driverLeft, CommandGenericHID driverRight, CommandGenericHID operator) {
        new TriggerBuilder<>(m_driverSubmap)
            .onTrue(driverRight.button(2), new InstantCommand(() -> Drive.getInstance().zeroHeading()))

            .whileTrue(driverLeft.button(0), new IntakeIn())
            .whileTrue(driverRight.button(0), new IntakeAAAAA())

            .whileTrue(driverLeft.button(2), new SlowSwerveDrive(m_driveInputs))

            .register();

        Pose3d leftTrench = FieldUtils.getInstance().getTagPose(FieldUtils.getInstance().getAllianceAprilTags().frontLeftTrench);
        Pose3d rightTrench = FieldUtils.getInstance().getTagPose(FieldUtils.getInstance().getAllianceAprilTags().frontRightTrench);

        Pose3d leftClose = leftTrench.plus(new Transform3d(new Pose3d(), new Pose3d(new Translation3d(2.3, 1, 0), Rotation3d.kZero)));
        Pose3d leftFar = leftTrench.plus(new Transform3d(new Pose3d(), new Pose3d(new Translation3d(-2.3, 1, 0), Rotation3d.kZero)));
        Pose3d rightClose = rightTrench.plus(new Transform3d(new Pose3d(), new Pose3d(new Translation3d(2.3, -1, 0), Rotation3d.kZero)));
        Pose3d rightFar = rightTrench.plus(new Transform3d(new Pose3d(), new Pose3d(new Translation3d(-2.3, -1, 0), Rotation3d.kZero)));

        new TriggerBuilder<Submap>(m_operatorSubmap)
            .whileTrue(new Trigger(() -> {
                if ((operator.getHID().getRawButton(5) && m_lastStrumSide) ||
                    (operator.getHID().getRawButton(6) && !m_lastStrumSide)) {
                    m_timeSinceStrum = Timer.getFPGATimestamp();
                    m_lastStrumSide = !m_lastStrumSide;
                }
                return m_timeSinceStrum < 0.4;
            }), new SpindexerSpin())

            .whileTrue(operator.button(0), new ShootToPose(FieldUtils.getInstance()::getHubPose))

            .whileTrue(operator.button(1), new ShootToPose(() -> leftClose))
            .whileTrue(operator.button(2), new ShootToPose(() -> leftFar))
            .whileTrue(operator.button(3), new ShootToPose(() -> rightClose))
            .whileTrue(operator.button(4), new ShootToPose(() -> rightFar))

            .onTrue(
                operator.button(0)
                    .and(operator.button(2))
                    .and(operator.button(4)),
                new CalibrateTurretFull())

            .onTrue(
                operator.button(1)
                    .and(operator.button(3)),
                new HoodCalibrate())

            .register();
    }

    public GenericHID getDriverLeftController() {
        return m_driverLeft.getHID();
    }

    public GenericHID getDriverRightController() {
        return m_driverRight.getHID();
    }

    public GenericHID getOperatorGuitarController() {
        return m_operatorGuitar.getHID();
    }

    public SwerveDriveInputs getDriveInputs() {
        return m_driveInputs;
    }
}
