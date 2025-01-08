package frc.robot.subsystems.elevator;

public interface ElevatorIO {
    public void setSpeed(double speed);
    public void setPosition(double position);
    public double getSpeed();
    public double getPosition();
}
