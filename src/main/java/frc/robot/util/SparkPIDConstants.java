package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;

public class SparkPIDConstants {
    public static final NetworkTable constantPreferences = NetworkTableInstance.getDefault().getTable("Custom Robot Preferences");
    public static boolean enableNT = true;

    public final String name;
    public final double kDefaultP;
    public final double kDefaultI;
    public final double kDefaultD;
    public final double kDefaultV;

    public SparkPIDConstants(String name, double kDefaultP, double kDefaultI, double kDefaultD, double kDefaultV) {
        this.name = name;
        this.kDefaultP = kDefaultP;
        this.kDefaultI = kDefaultI;
        this.kDefaultD = kDefaultD;
        this.kDefaultV = kDefaultV;
        
        if(!enableNT) return;
        Preferences.initDouble("k" + name + "P", this.kDefaultP);
        Preferences.initDouble("k" + name + "I", this.kDefaultI);
        Preferences.initDouble("k" + name + "D", this.kDefaultD);
        Preferences.initDouble("k" + name + "V", this.kDefaultV);

        System.out.println(name + " preferences have been initialized");
    }

    public void resetPreferences() {
        if(!enableNT) return;
        Preferences.setDouble("k" + name + "P", kDefaultP);
        Preferences.setDouble("k" + name + "I", kDefaultI);
        Preferences.setDouble("k" + name + "D", kDefaultD);
        Preferences.setDouble("k" + name + "V", kDefaultV);
    }

    public double kP() {
        if(!enableNT) return kDefaultP;
        return Preferences.getDouble("k" + name + "P", kDefaultP);
    }

    public double kI() {
        if(!enableNT) return kDefaultI;
        return Preferences.getDouble("k" + name + "I", kDefaultI);
    }

    public double kD() {
        if(!enableNT) return kDefaultD;
        return Preferences.getDouble("k" + name + "D", kDefaultD);
    }

    public double kV() {
        if(!enableNT) return kDefaultV;
        return Preferences.getDouble("k" + name + "V", kDefaultV);
    }
}
