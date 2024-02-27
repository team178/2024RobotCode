package frc.robot.subsystems.intake;

public enum IntakePosition {
    HOME(0),
    DEPLOYED(0);
    
    public final double position;
    
    private IntakePosition(double position) {
        this.position = position;
    }
}
