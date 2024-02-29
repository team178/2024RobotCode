package frc.robot.subsystems.shooter;

public enum ShooterPosition {
    FLAT(18.5),
    AMP(108.7),
    SPEAKER(65),
    SOURCE(49);
    
    public final double position;
    
    private ShooterPosition(double position) {
        this.position = position;
    }
}
