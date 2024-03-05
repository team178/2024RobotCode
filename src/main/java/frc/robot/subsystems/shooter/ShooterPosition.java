package frc.robot.subsystems.shooter;

public enum ShooterPosition {
    FLAT("FLAT", 19.5),
    AMP("AMP", 90), // prev 108.7
    SPEAKER("SPEAKER", 62.9),
    SOURCE("SOURCE", 49);
    // amp or speaker shooting method adjusted if needed could work for trap
    
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
