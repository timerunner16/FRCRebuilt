// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.testingdashboard;

import java.util.ArrayList;

import frc.robot.utils.Configuration;
import frc.robot.utils.structlogging.StructLogger;

public class SubsystemBase extends edu.wpi.first.wpilibj2.command.SubsystemBase {

  ArrayList<TDValue> m_values;
  ArrayList<StructLogger> m_structLoggers;
  TDBoolean m_enabled;
  /** Creates a new Subsystem and registers it with the TestingDashboard. */
  public SubsystemBase(String name)
  {
    m_values = new ArrayList<TDValue>();
    m_structLoggers = new ArrayList<StructLogger>();
    setName(name);
    TestingDashboard.getInstance().registerTab(name);
    // must do this after the subsystem is registered
    m_enabled = new TDBoolean(this, "TestingDashboard", "Enabled", true);
  }

  @Override
  public void periodic() {

    if (!m_enabled.get()) return;

    for(TDValue value : m_values)
    {
      value.post();
    }

    for (StructLogger structLogger : m_structLoggers) {
      structLogger.post();
    }
  }

  public void registerValue(TDValue value) {
    m_values.add(value);
  }

  public void registerStructLogger(StructLogger structLogger) {
    m_structLoggers.add(structLogger);
  }

  public Configuration config()
  {
    return Configuration.getInstance();
  }

  public double cfgDbl(String name)
  {
    return config().getDouble(getName(), name);
  }

  public int cfgInt(String name)
  {
    return config().getInt(getName(), name);
  }

  public boolean cfgBool(String name)
  {
    return config().getBool(getName(), name);
  }
}