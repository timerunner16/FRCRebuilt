package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.SwerveDriveInputs;

public class OI {
    private static OI m_OI;

    private XboxController m_driverXboxController;
    private XboxController m_operatorXboxController;

    private SwerveDriveInputs m_driveInputs;

    public static OI getInstance() {
        if (m_OI == null) m_OI = new OI();
        return m_OI;
    }

    private OI() {
        m_driverXboxController = new XboxController(RobotMap.U_DRIVER_XBOX_CONTROLLER);
        m_operatorXboxController = new XboxController(RobotMap.U_OPERATOR_XBOX_CONTROLLER);

        Supplier<Double> xInput;
        Supplier<Double> yInput;
        if (RobotBase.isReal()) {
            xInput = ()->m_driverXboxController.getLeftY();
            yInput = ()->m_driverXboxController.getLeftX();
        } else {
            xInput = ()->-m_driverXboxController.getLeftX();
            yInput = ()->m_driverXboxController.getLeftY();
        }
        m_driveInputs = new SwerveDriveInputs(xInput, yInput, ()->m_driverXboxController.getRightX());
    }

    public void bindControls() {
        
    }

    public XboxController getDriverController() {
        return m_driverXboxController;
    }

    public XboxController getOperatorController() {
        return m_operatorXboxController;
    }

    public SwerveDriveInputs getDriveInputs() {
        return m_driveInputs;
    }
}
