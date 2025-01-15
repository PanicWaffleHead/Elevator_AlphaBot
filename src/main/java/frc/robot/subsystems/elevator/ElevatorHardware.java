package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class ElevatorHardware implements ElevatorIO {

    public static final class ElevatorHardwareConstants {
        private static final int LEFT_ELEVATOR_MOTOR_ID = 7;
        private static final int RIGHT_ELEVATOR_MOTOR_ID = 10;
        private static final double METERS_PER_REVOLUTION = Units.inchesToMeters(27) / 41.951946;
        private static final Distance ALLOWED_SETPOINT_ERROR = Inches.of(1); 
        private static final LinearVelocity MAX_VEL = MetersPerSecond.of(0.8);
        private static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(0.4);
        private static final double P_VALUE = 0.1;
        private static final double I_VALUE = 0;
        private static final double D_VALUE = 0.1;
        private static final double FEEDFORWARD_VALUE = 1.0 / 917;
        private static final double OUTPUTRANGE_MIN_VALUE = -1;
        private static final double OUTPUTRANGE_MAX_VALUE = 1;
        private static final double P_VALUE_VELOCITY = 0.0001;
        private static final double I_VALUE_VELOCITY = 0;
        private static final double D_VALUE_VELOCITY = 0;
    }

    public SparkMax elevatorRightMotorLeader, elevatorLeftMotorFollower;
    private SparkMaxConfig globalMotorConfig, rightMotorConfigLeader, leftMotorConfigFollower;
    private SparkClosedLoopController rightClosedLoopController;
    private RelativeEncoder rightEncoder;
    private double position;
    
    public ElevatorHardware() {

        globalMotorConfig = new SparkMaxConfig();
        rightMotorConfigLeader = new SparkMaxConfig();
        leftMotorConfigFollower = new SparkMaxConfig();

        rightEncoder = elevatorRightMotorLeader.getEncoder();

        elevatorRightMotorLeader = new SparkMax(ElevatorHardwareConstants.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        elevatorLeftMotorFollower = new SparkMax(ElevatorHardwareConstants.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        rightClosedLoopController = elevatorRightMotorLeader.getClosedLoopController();

        globalMotorConfig.encoder
            .positionConversionFactor(ElevatorHardwareConstants.METERS_PER_REVOLUTION)
            .velocityConversionFactor(ElevatorHardwareConstants.METERS_PER_REVOLUTION / 60);
            
        globalMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(ElevatorHardwareConstants.P_VALUE, ClosedLoopSlot.kSlot0)
            .i(ElevatorHardwareConstants.I_VALUE, ClosedLoopSlot.kSlot0)
            .d(ElevatorHardwareConstants.D_VALUE, ClosedLoopSlot.kSlot0)
            .outputRange(-1, 1)
            .p(ElevatorHardwareConstants.P_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
            .i(ElevatorHardwareConstants.I_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
            .d(ElevatorHardwareConstants.D_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
            //https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started#f-parameter
            .velocityFF(ElevatorHardwareConstants.FEEDFORWARD_VALUE, ClosedLoopSlot.kSlot1) 
            .outputRange(ElevatorHardwareConstants.OUTPUTRANGE_MIN_VALUE, ElevatorHardwareConstants.OUTPUTRANGE_MAX_VALUE, ClosedLoopSlot.kSlot1);

        globalMotorConfig.closedLoop.maxMotion
            .maxVelocity(ElevatorHardwareConstants.MAX_VEL.in(MetersPerSecond))
            .maxAcceleration(ElevatorHardwareConstants.MAX_ACCEL.in(MetersPerSecondPerSecond))
            .allowedClosedLoopError(ElevatorHardwareConstants.ALLOWED_SETPOINT_ERROR.in(Meters));

        rightMotorConfigLeader.apply(globalMotorConfig).inverted(true);

        leftMotorConfigFollower.apply(globalMotorConfig).follow(elevatorRightMotorLeader);

        elevatorLeftMotorFollower.configure(leftMotorConfigFollower, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        elevatorRightMotorLeader.configure(rightMotorConfigLeader, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setSpeed(double speed) {
        //rightClosedLoopController.setReference(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        rightClosedLoopController.setReference(speed, SparkBase.ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void setPosition(double position) {
        //rightClosedLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        rightClosedLoopController.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl);

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
