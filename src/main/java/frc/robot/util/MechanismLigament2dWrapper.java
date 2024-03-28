package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class MechanismLigament2dWrapper {
    // Made to be able to access current values of a ligament 2d if needed
    public final MechanismLigament2d ligament;
    private double angle;
    private Color8Bit color;
    private double length;
    private double lineWidth;

    public MechanismLigament2dWrapper(String name, double length, double angle, double lineWidth, Color8Bit color) {
        ligament = new MechanismLigament2d(name, length, angle, lineWidth, color);
        this.angle = angle;
        this.color = color;
        this.length = length;
        this.lineWidth = lineWidth;
    }

    public void setAngle(double angle) {
        ligament.setAngle(angle);
        this.angle = angle;
    }

    public void setAngle(Rotation2d angle) {
        ligament.setAngle(angle);
        this.angle = angle.getDegrees();
    }
    
    public void setColor(Color8Bit color) {
        ligament.setColor(color);
        this.color = color;
    }

    public void setLength(double length) {
        if(length < 0) {
            ligament.setAngle(angle - 180);
        }
        ligament.setLength(length);
        this.length = length;
    }

    public void setLineWidth(double lineWidth) {
        ligament.setLineWeight(lineWidth);
        this.lineWidth = lineWidth;
    }
}
