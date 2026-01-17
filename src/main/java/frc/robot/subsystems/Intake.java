package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.RobotMap;
import frc.robot.testingdashboard.TDNumber;
import frc.robot.testingdashboard.SubsystemBase;

public class Intake extends SubsystemBase{

    SparkMax m_ISparkMax;
    SparkMaxConfig m_SparkMaxConfig;

    private static Intake m_Intake = null;

    private Intake(){
        super("Intake");
        if (RobotMap.I_ENABLED == true){
            m_ISparkMax = new SparkMax(RobotMap.I_MOTOR_CAN_ID, MotorType.kBrushless); //unsure if we are using brushed or brushless but we used brushless on leaflet
            m_SparkMaxConfig = new SparkMaxConfig();
            m_SparkMaxConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(25, 60); //settings from offseason robot, unsure what they should be
                
        }
    }

    public static Intake getInstance() {
        if (m_Intake == null) {
            m_Intake = new Intake();
        }
        return m_Intake;
    }
    
}
