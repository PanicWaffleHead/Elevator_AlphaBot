package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorConstants;

public class ElevatorHardware implements ElevatorIO {
    public SparkMax elevatorRightMotorLeader, elevatorLeftMotorFollower;
    private SparkMaxConfig globalMotorConfig, rightMotorConfigLeader, leftMotorConfigFollower;
    private SparkClosedLoopController rightClosedLoopController, leftClosedLoopController;
    private RelativeEncoder rightEncoder;
    private double position;
    
    public ElevatorHardware() {
        elevatorRightMotorLeader = new SparkMax(ElevatorConstants.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        elevatorLeftMotorFollower = new SparkMax(ElevatorConstants.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        rightClosedLoopController = elevatorRightMotorLeader.getClosedLoopController();
        leftClosedLoopController = elevatorLeftMotorFollower.getClosedLoopController();

        rightEncoder = elevatorRightMotorLeader.getEncoder();

        globalMotorConfig = new SparkMaxConfig();
        rightMotorConfigLeader = new SparkMaxConfig();
        leftMotorConfigFollower = new SparkMaxConfig();

        globalMotorConfig.encoder
            .positionConversionFactor(ElevatorConstants.METERS_PER_REVOLUTION)
            .velocityConversionFactor(ElevatorConstants.METERS_PER_REVOLUTION / 60);
            

        globalMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        //https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started#f-parameter
        .velocityFF(1.0 / 917, ClosedLoopSlot.kSlot1) 
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        globalMotorConfig.closedLoop.maxMotion
            .maxVelocity(ElevatorConstants.MAX_VEL.in(MetersPerSecond))
            .maxAcceleration(ElevatorConstants.MAX_ACCEL.in(MetersPerSecondPerSecond))
            .allowedClosedLoopError(ElevatorConstants.ALLOWED_SETPOINT_ERROR.in(Meters));

        rightMotorConfigLeader.apply(globalMotorConfig).inverted(true);

        leftMotorConfigFollower.apply(globalMotorConfig).follow(elevatorRightMotorLeader);

        elevatorLeftMotorFollower.configure(leftMotorConfigFollower, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        elevatorRightMotorLeader.configure(rightMotorConfigLeader, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setSpeed(double speed) {
        rightClosedLoopController.setReference(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        leftClosedLoopController.setReference(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }

    @Override
    public void setPosition(double position) {
        rightClosedLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        leftClosedLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);

        this.position = position;
    }

    @Override
    public double getVelocity() {
        return rightEncoder.getVelocity();
    }

    @Override
    public double getPosition() {
        return rightEncoder.getPosition();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = getPosition();
        inputs.velocity = getVelocity();
        inputs.appliedVoltage = elevatorRightMotorLeader.getAppliedOutput() * 12;
        inputs.positionSetPoint = position;
    }
}
