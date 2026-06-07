package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.SetX;
import frc.robot.commands.drive.SlowSwerveDrive;
import frc.robot.commands.intake.DeployerIn;
import frc.robot.commands.intake.DeployerOut;
import frc.robot.commands.intake.IntakeAAAAA;
import frc.robot.commands.intake.IntakeIn;
import frc.robot.commands.intake.IntakeOscillate;
import frc.robot.commands.shooter.CalibrateTurretFull;
import frc.robot.commands.shooter.HoodCalibrate;
import frc.robot.commands.shooter.ShootMap;
import frc.robot.commands.shooter.ShootToPose;
import frc.robot.commands.spindexer.SpindexerSpin;
import frc.robot.subsystems.Drive;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.Referrable;
import frc.robot.utils.TriggerBuilder;
import frc.robot.utils.TriggerBuilder.RumbleIndicator;
import frc.robot.utils.TriggerBuilder.SwitchIndicator;
import frc.robot.utils.drive.SwerveDriveInputs;

public class OI {
    private static OI m_OI;

    private CommandXboxController m_driver;
    private CommandGenericHID m_operatorGuitar;

    private SwerveDriveInputs m_driveInputs;

    private boolean m_lastStrumSide;
    private double m_lastStrumTime;

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
        m_lastStrumTime = Double.MAX_VALUE;
        m_lastStrumSide = false;

        m_driver = new CommandXboxController(RobotMap.U_DRIVER_XBOX_CONTROLLER);
        m_operatorGuitar = new CommandGenericHID(RobotMap.U_OPERATOR_GUITAR_CONTROLLER);

        Supplier<Double> xInput;
        Supplier<Double> yInput;
        if (RobotBase.isReal()) {
            xInput = () -> m_driver.getLeftY();
            yInput = () -> m_driver.getLeftX();
        } else {
            xInput = () -> -m_driver.getLeftY();
            yInput = () -> m_driver.getLeftX();
        }
        m_driveInputs = new SwerveDriveInputs(xInput, yInput, () -> m_driver.getRightX());
    }

    public void bindControls() {
        bindRockstarLayout(m_driver, m_operatorGuitar);
    }

    public void bindRockstarLayout(CommandXboxController driver, CommandGenericHID operator) {
        new TriggerBuilder<>(m_driverSubmap)
            .onTrue(driver.back(), new InstantCommand(() -> Drive.getInstance().zeroHeading()))

            .whileTrue(driver.leftBumper(), new IntakeIn())
            .whileTrue(driver.povUp(), new IntakeOscillate())
            .whileTrue(driver.rightBumper(), new IntakeAAAAA())
            .whileTrue(driver.b(), new DeployerOut())
            .whileTrue(driver.a(), new SetX())
            .whileTrue(driver.rightTrigger(), new DeployerIn())

            .whileTrue(driver.leftTrigger(), new SlowSwerveDrive(m_driveInputs))

            .register();

        Pose3d leftTrench = FieldUtils.getInstance().getTagPose(FieldUtils.getInstance().getAllianceAprilTags().frontLeftTrench);
        Pose3d rightTrench = FieldUtils.getInstance().getTagPose(FieldUtils.getInstance().getAllianceAprilTags().frontRightTrench);

        Pose3d leftClose = leftTrench.plus(new Transform3d(new Pose3d(), new Pose3d(new Translation3d(2.3, 1, 0), Rotation3d.kZero)));
        Pose3d leftFar = leftTrench.plus(new Transform3d(new Pose3d(), new Pose3d(new Translation3d(-2.3, 1, 0), Rotation3d.kZero)));
        Pose3d rightClose = rightTrench.plus(new Transform3d(new Pose3d(), new Pose3d(new Translation3d(2.3, -1, 0), Rotation3d.kZero)));
        Pose3d rightFar = rightTrench.plus(new Transform3d(new Pose3d(), new Pose3d(new Translation3d(-2.3, -1, 0), Rotation3d.kZero)));

        Trigger strumming = new Trigger(() -> {
            if ((operator.getHID().getRawButton(6) && m_lastStrumSide) ||
                (operator.getHID().getRawButton(7) && !m_lastStrumSide)) {
                    m_lastStrumTime = Timer.getFPGATimestamp();
                    m_lastStrumSide = !m_lastStrumSide;
            }
            if (Math.abs(Timer.getFPGATimestamp() - m_lastStrumTime) < 0.4) {
                System.out.println("Strumming");
                return true;
            }
            return false;
        });

        new TriggerBuilder<Submap>(m_operatorSubmap)
            .onTrue(
                operator.button(1)
                .and(operator.button(3))
                .and(operator.button(5))
                .and(strumming),
                new CalibrateTurretFull())

            .onTrue(
                operator.button(2)
                .and(operator.button(4))
                .and(strumming),
                new HoodCalibrate())

            .beginSubmap(Submap.AUTO)
                .whileTrue(
                    operator.button(1).and(strumming), 
                    Commands.parallel(
                        new ShootToPose(FieldUtils.getInstance()::getHubPose),
                        new SpindexerSpin()))
                .whileTrue(
                    operator.button(2).and(strumming), 
                    Commands.parallel(
                        new ShootToPose(() -> leftClose),
                        new SpindexerSpin()))
                .whileTrue(
                    operator.button(3).and(strumming), 
                    Commands.parallel(
                        new ShootToPose(() -> leftFar),
                        new SpindexerSpin()))
                .whileTrue(
                    operator.button(4).and(strumming), 
                    Commands.parallel(
                        new ShootToPose(() -> rightClose),
                        new SpindexerSpin()))
                .whileTrue(
                    operator.button(5).and(strumming), 
                    Commands.parallel(
                        new ShootToPose(() -> rightFar),
                        new SpindexerSpin()))

                .switchSubmap(null,
                        operator.button(1)
                        .and(operator.button(2))
                        .and(operator.button(3))
                        .and(operator.button(4))
                        .and(strumming), Submap.MANUAL)
            .endSubmap()

            .beginSubmap(Submap.MANUAL)
                // left trench
                .whileTrue(
                    operator.button(5).and(strumming),
                    Commands.parallel(
                        new ShootMap(
                            new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(4.32, 7.5), Rotation2d.kCW_90deg),
                                    3000, -21.8, 0.13),
                            new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(4.32, 7.5), Rotation2d.kCW_90deg),
                                    3000, -21.8, 0.13),
                            Alliance.Blue,
                            ShootMap.Target.SHOOT_HUB),
                        new SpindexerSpin()))

                // left corner
                .whileTrue(
                    operator.button(4).and(strumming),
                    Commands.parallel(
                        new ShootMap(
                            new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(0.5, 7.5), Rotation2d.kZero), 3400,
                                    -35, -0.77),
                            new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(0.5, 7.5), Rotation2d.kZero), 3400,
                                    -35, -0.77),
                            Alliance.Blue,
                            ShootMap.Target.SHOOT_HUB),
                        new SpindexerSpin()))

                // ladder
                .whileTrue(
                    operator.button(3).and(strumming),
                    Commands.parallel(
                        new ShootMap(
                            new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(1.5, 4), Rotation2d.kZero), 3100,
                                    -25, 0),
                            new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(1.5, 4), Rotation2d.kZero), 3100,
                                    -25, 0),
                            Alliance.Blue,
                            ShootMap.Target.SHOOT_HUB),
                        new SpindexerSpin()))

                // right corner
                .whileTrue(
                    operator.button(2).and(strumming),
                    Commands.parallel(
                        new ShootMap(
                            new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(0.5, 0.5), Rotation2d.kZero), 3400,
                                    -35, 0.59),
                            new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(0.5, 0.5), Rotation2d.kZero), 3400,
                                    -35, 0.59),
                            Alliance.Blue,
                            ShootMap.Target.SHOOT_HUB),
                        new SpindexerSpin()))

                // right trench
                .whileTrue(
                    operator.button(1).and(strumming),
                    Commands.parallel(
                        new ShootMap(
                            new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(4.32, 7.5), Rotation2d.kCCW_90deg),
                                    3000, -21.8, -0.35),
                            new ShootMap.ShootMapSetpoint(new Pose2d(new Translation2d(4.32, 7.5), Rotation2d.kCCW_90deg),
                                    3000, -21.8, -0.35),
                            Alliance.Blue,
                            ShootMap.Target.SHOOT_HUB),
                        new SpindexerSpin()))
                

                .switchSubmap(null,
                    operator.button(1)
                        .and(operator.button(2))
                        .and(operator.button(3))
                        .and(operator.button(4))
                        .and(strumming),
                    Submap.AUTO)
            .endSubmap()

            .register();
    }

    public XboxController getDriverLeftController() {
        return m_driver.getHID();
    }

    public GenericHID getOperatorGuitarController() {
        return m_operatorGuitar.getHID();
    }

    public SwerveDriveInputs getDriveInputs() {
        return m_driveInputs;
    }
}
