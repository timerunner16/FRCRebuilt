package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Shooter;
import frc.robot.testingdashboard.Command;

public class ManualShooterControl extends Command{
    private final Shooter m_Shooter;
    private final OI m_OI;

    private double m_hoodAngle, m_flywheelSpeed, m_turretAngle;

    public ManualShooterControl() {
        super(Shooter.getInstance(), "Controls", "ManualShooterControl");

        m_Shooter = Shooter.getInstance();
        m_OI = OI.getInstance();
        
        addRequirements(m_Shooter);
    }

    @Override
    public void initialize() {
        m_hoodAngle = 0;
        m_flywheelSpeed = 0;
        m_turretAngle = m_Shooter.getTurretTarget();
    }

    @Override
    public void execute() {
        XboxController operator = m_OI.getOperatorController();

        m_turretAngle += operator.getLeftX() * Constants.schedulerPeriodTime;
        m_turretAngle %= 360;
        m_hoodAngle += operator.getRightY() * Constants.schedulerPeriodTime;
        m_hoodAngle = Math.max(Math.min(m_hoodAngle, 0), -40);

        int pov = operator.getPOV();

        m_flywheelSpeed +=
            ((pov == 0) ? Constants.schedulerPeriodTime : 0) -
            ((pov == 180) ? Constants.schedulerPeriodTime : 0);
        m_flywheelSpeed = Math.min(Math.max(m_flywheelSpeed, 0), 6000);

        m_Shooter.setFlywheelTarget(m_flywheelSpeed);
        m_Shooter.setTurretTarget(m_turretAngle, 0);
        m_Shooter.setHoodTarget(m_hoodAngle);
    }
}
