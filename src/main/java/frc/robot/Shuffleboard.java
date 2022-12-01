
package frc.robot;

import java.util.HashMap;
import java.util.Map;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;

public class Shuffleboard {
    private NetworkTableEntry numValue;
    private NetworkTableEntry stringValue;
    private NetworkTableEntry booleanValue;
    private Map<String, NetworkTableEntry> networkTableNameMap = new HashMap<String, NetworkTableEntry>();
    private String currnetTab;

    public Shuffleboard(String tab) {
        currnetTab = tab;

    }

    private NetworkTableEntry shuffleboardPattern(NetworkTableEntry valEntry, String title, Object dv) {
        if (networkTableNameMap.get(title) == null) {
            valEntry = edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab(currnetTab).add(title, dv).getEntry();
            networkTableNameMap.put(title, valEntry);
        }
        return networkTableNameMap.get(title);
    }

    private NetworkTableEntry shuffleboardPattern(NetworkTableEntry valEntry, String title, Object DV,
            WidgetType widgetType) {
        if (networkTableNameMap.get(title) == null) {
            valEntry = edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab(currnetTab).add(title, DV)
                    .withWidget(widgetType).getEntry();
            networkTableNameMap.put(title, valEntry);
        }
        return networkTableNameMap.get(title);
    }

    public void addNum(String title, double value) {
        shuffleboardPattern(numValue, title, 0).setValue(value);
    }

    public void addString(String title, String value) {
        shuffleboardPattern(stringValue, title, "").setValue(value);
    }

    public void addBoolean(String title, Boolean value) {
        shuffleboardPattern(booleanValue, title, false).setValue(value);
    }

    public void addNum(String title, double value, WidgetType widgetType) {
        shuffleboardPattern(numValue, title, 0, widgetType).setValue(value);
    }

    public void addString(String title, String value, WidgetType widgetType) {
        shuffleboardPattern(stringValue, title, "", widgetType).setValue(value);
    }

    public void addBoolean(String title, Boolean value, WidgetType widgetType) {
        shuffleboardPattern(booleanValue, title, false, widgetType).setValue(value);
    }

    public boolean getBoolean(String title) {

        return shuffleboardPattern(booleanValue, title, false).getBoolean(false);
    }

    public String getString(String title) {
        return shuffleboardPattern(stringValue, title, "").getString("");
    }

    public double getNum(String title) {
        return (double) shuffleboardPattern(numValue, title, 0).getNumber(0);
    }

    public boolean getBoolean(String title, WidgetType widgetType) {
        return shuffleboardPattern(booleanValue, title, false, widgetType).getBoolean(false);
    }

    public String getString(String title, WidgetType widgetType) {
        return shuffleboardPattern(stringValue, title, "", widgetType).getString("");
    }

    public double getNum(String title, WidgetType widgetType) {
        return (double) shuffleboardPattern(numValue, title, 0, widgetType).getNumber(0);
    }
}