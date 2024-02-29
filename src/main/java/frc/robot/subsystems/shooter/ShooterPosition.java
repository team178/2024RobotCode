package frc.robot.subsystems.shooter;

public enum ShooterPosition {
    FLAT("FLAT", 18.5),
    AMP("AMP", 108.7),
    SPEAKER("SPEAKER", 63.5),
    SOURCE("SOURCE", 49);
    
    public final double position;
    public final String name;
    
    private ShooterPosition(String name, double position) {
        this.name = name;
        this.position = position;
    }

    @Override
    public String toString() {
        return name;
    }
}
