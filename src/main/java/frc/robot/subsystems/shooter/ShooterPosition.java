package frc.robot.subsystems.shooter;

public enum ShooterPosition {
    FLAT(0),
    AMP(0),
    SPEAKER(0),
    UP(0); //might use SPEAKER position instead?
    
    public final double position;
    
    private ShooterPosition(double position) {
        this.position = position;
    }
}
