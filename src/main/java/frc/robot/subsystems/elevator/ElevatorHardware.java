package frc.robot.subsystems.elevator;

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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorConstants;

public class ElevatorHardware implements ElevatorIO {
    public SparkMax elevatorRightMotorLeader, elevatorLeftMotorFollower;
    private SparkMaxConfig globalMotorConfig, rightMotorConfigLeader, leftMotorConfigFollower;
    private SparkClosedLoopController rightClosedLoopController;
    private SparkClosedLoopController leftClosedLoopController;
    private RelativeEncoder rightEncoder;
    
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
            .maxVelocity(ElevatorConstants.MAX_VEL)
            .maxAcceleration(ElevatorConstants.MAX_ACCEL)
            .allowedClosedLoopError(ElevatorConstants.ALLOWED_SETPOINT_ERROR);

        rightMotorConfigLeader.apply(globalMotorConfig).inverted(true);

        leftMotorConfigFollower.apply(globalMotorConfig).follow(elevatorRightMotorLeader);

        elevatorLeftMotorFollower.configure(leftMotorConfigFollower, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        elevatorRightMotorLeader.configure(rightMotorConfigLeader, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.setDefaultNumber("Target Position", 0);
        SmartDashboard.setDefaultNumber("Target Velocity", 0);
        SmartDashboard.setDefaultBoolean("Control Mode", false);
        SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    }

    public void setSpeed(double speed) {

    }

    public void setPosition(double position) {
        rightClosedLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        leftClosedLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public double getSpeed() {
        return rightEncoder.getVelocity();
    }

    public double getPosition() {
        return rightEncoder.getPosition();
    }
}
