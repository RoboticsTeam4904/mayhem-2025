package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.usfirst.frc4904.standard.commands.Noop;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;

public class ElevatorSubsystem extends MultiMotorSubsystem {

    // TODO: tune
    public static final double kS = 0.00;
    public static final double kV = 1.4555;
    public static final double kA = 0.0513;
    public static final double kG = 0.235;

    public static final double kP = 0.07;
    public static final double kI = 0.03;
    public static final double kD = 0;

    public final ArmFeedforward feedforward; // TODO not an arm anymore
    public final Encoder encoder;

    // make sure that all values defined in this enum are added to the 'positions' map in the constructor
    public enum Position {
        INTAKE,
        OUTTAKE,
    }

    public static HashMap<Position, Double> positions = new HashMap<>();

    // possible helpful https://www.chiefdelphi.com/t/using-encoder-to-drive-a-certain-distance/147219/2
    public ElevatorSubsystem(CANTalonFX motor1, CANTalonFX motor2, Encoder encoder) {
        super(
            new CANTalonFX[] { motor1, motor2 },
            new double[] { 1, -1 },
            0 // if we ever want to have up/down commands that use a set voltage in addition to PID, put that voltage here
        );
        this.feedforward = new ArmFeedforward(kG, kS, kV, kA);
        this.encoder = encoder;
        encoder.setDistancePerPulse(5);

        // TODO change (obviously)
        positions.put(Position.INTAKE, 0.0);
        positions.put(Position.OUTTAKE, 4.0);
    }

    // public double GetDistance() {
    //     return encoder.getDistance();
    // }

    // public Command c_controlAngularVelocity(DoubleSupplier degPerSecDealer) {
    //     // TODO
    //     // var cmd = this.run(() -> {
    //     //     var ff = this.feedforward.calculate(
    //     //         Units.degreesToRadians(getCurrentAngleDegrees()),
    //     //         Units.degreesToRadians(degPerSecDealer.getAsDouble())
    //     //     );
    //     //     SmartDashboard.putNumber("feedforward", ff);
    //     //     this.setVoltage(ff);
    //     // });
    //     // cmd.setName("arm - c_controlAngularVelocity");
    //     //
    //     // return cmd;
    // }

    // public Command c_gotoPosition(Position pos) {
    //     return c_gotoPosition(pos, null);
    // }

    // public Command c_gotoPosition(Position pos, Supplier<Command> onArrivalCommandDealer) {
    //     Double height = positions.get(pos);

    //     if (height == null) {
    //         System.err.println(
    //             "ElevatorSubsystem.Position." + pos.name() + " is not defined in 'positions' map"
    //         );
    //         return new Noop();
    //     }

    //     return c_gotoHeight(height, onArrivalCommandDealer);
    // }

    // public Command c_gotoHeight(double height, Supplier<Command> onArrivalCommandDealer) {
    //     // TODO
    //     // ezControl controller = new ezControl(
    //     //     kP, kI, kD,
    //     //     (position, velocityDegPerSec) -> {
    //     //         double ff = this.feedforward.calculate(
    //     //             Units.degreesToRadians(getCurrentAngleDegrees()),
    //     //             Units.degreesToRadians(velocityDegPerSec)
    //     //         );
    //     //         SmartDashboard.putNumber("Intended voltage", maxAccelDegPerSecSquare);
    //     //         return ff;
    //     //     }
    //     // );
    //     //
    //     // TrapezoidProfile profile = new TrapezoidProfile(
    //     //     new TrapezoidProfile.Constraints(maxVelDegPerSec, maxAccelDegPerSecSquare)
    //     // );
    //     //
    //     // var cmd = getEzMotion(
    //     //     controller,
    //     //     profile,
    //     //     new TrapezoidProfile.State(degreesFromHorizontal, 0),
    //     //     new TrapezoidProfile.State(getCurrentAngleDegrees(), 0)
    //     // );
    //     //
    //     // return onArrivalCommandDealer == null
    //     //     ? cmd
    //     //     : Utils.nameCommand(
    //     //         "pivot w/ onArrival: " + cmd.getName(),
    //     //         new ParallelCommandGroup(
    //     //             cmd,
    //     //             new SequentialCommandGroup(
    //     //                 new WaitCommand(profile.totalTime()),
    //     //                 new CreateAndDisown("arm pivot", onArrivalCommandDealer)
    //     //             )
    //     //         )
    //     //     );
    // }

    // private ezMotion getEzMotion(
    //     ezControl controller,
    //     TrapezoidProfile profile,
    //     TrapezoidProfile.State current,
    //     TrapezoidProfile.State goal
    // ) {
    //     // TODO
    //     // var cmd = new ezMotion(
    //     //     controller,
    //     //     this::getCurrentAngleDegrees,
    //     //     (double volts) -> {
    //     //         SmartDashboard.putNumber("Arm Volts", volts);
    //     //         this.setVoltage(volts);
    //     //     },
    //     //     (double t) -> {
    //     //         TrapezoidProfile.State result = profile.calculate(t, current, goal);
    //     //
    //     //         SmartDashboard.putNumber("deg setpoint", result.velocity);
    //     //         return new Pair<>(result.position, result.velocity);
    //     //     },
    //     //     this
    //     // );
    //     // cmd.setName("arm - c_holdRotation");
    //     //
    //     // return cmd;
    // }
}
