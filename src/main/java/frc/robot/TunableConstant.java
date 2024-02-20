package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TunableConstant {
  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("tuning");
  private final NetworkTableEntry m_dashboardEntry;
  private final double m_defaultValue;
  private double m_currentValue;
  
  public TunableConstant(String key, double defaultValue) {
    m_defaultValue = defaultValue;
    m_dashboardEntry = table.getEntry(key);
    m_dashboardEntry.setNumber(defaultValue);
  }

  public double get() {
    update();
    return m_currentValue;
  }

  private void update() {
    m_currentValue = m_dashboardEntry.getDouble(m_defaultValue);
  }

  private double peek() {
    return m_dashboardEntry.getDouble(m_defaultValue);
  }

  public boolean hasChanged() {
    double newValue = peek();
    return newValue != m_currentValue;
  }
}
