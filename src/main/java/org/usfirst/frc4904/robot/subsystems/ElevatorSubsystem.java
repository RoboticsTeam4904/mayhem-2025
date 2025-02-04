package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.usfirst.frc4904.robot.Utils;
import org.usfirst.frc4904.standard.commands.CreateAndDisown;
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

    public final double MAX_VEL = 1;
    public final double MAX_ACCEL = 1;

    public final double MIN_HEIGHT = 0;
    public final double MAX_HEIGHT = 5;

    public final ElevatorFeedforward feedforward;
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
        this.feedforward = new ElevatorFeedforward(kG, kS, kV, kA);
        this.encoder = encoder;

        // TODO what even is this
        encoder.setDistancePerPulse(5);

        // TODO change (obviously)
        positions.put(Position.INTAKE, 0.0);
        positions.put(Position.OUTTAKE, 4.0);

        for (var pos : Position.values()) {
            if (positions.get(pos) == null) {
                System.err.println(
                    "ElevatorSubsystem.Position." +
                    pos.name() +
                    " is not defined in 'positions' map"
                );
            }
        }
    }

    public double getDistance() {
        return encoder.getDistance();
    }

    public Command c_controlVelocity(DoubleSupplier metersPerSecDealer) {
        if (
            (this.getDistance() > MAX_HEIGHT && metersPerSecDealer.getAsDouble() > 0) ||
            (this.getDistance() < MIN_HEIGHT && metersPerSecDealer.getAsDouble() < 0)
        ) {
            return this.c_stop();
        }

        var cmd =
            this.run(() -> {
                    var ff = this.feedforward.calculate(metersPerSecDealer.getAsDouble());
                    SmartDashboard.putNumber("feedforward", ff);
                    this.setVoltage(ff);
                });
        cmd.setName("elevator - c_controlVelocity");

        return cmd;
    }

    public Command c_gotoPosition(Position pos) {
        return c_gotoPosition(pos, null);
    }

    public Command c_gotoPosition(Position pos, Supplier<Command> onArrivalCommandDealer) {
        Double height = positions.get(pos);

        if (height == null) return new Noop(); // not good

        return c_gotoHeight(height, onArrivalCommandDealer);
    }

    public Command c_gotoHeight(double height, Supplier<Command> onArrivalCommandDealer) {
        ezControl controller = new ezControl(kP, kI, kD, (position, velocityMetersPerSec) ->
            this.feedforward.calculate(velocityMetersPerSec)
        );

        TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL)
        );

        var cmd = getEzMotion(
            controller,
            profile,
            new TrapezoidProfile.State(getDistance(), 0), // TODO ???
            new TrapezoidProfile.State(height, 0)
        );
        cmd.setName("elevator - c_gotoHeight");

        return onArrivalCommandDealer == null
            ? cmd
            : Utils.nameCommand(
                "move elevator w/ onArrival: " + cmd.getName(),
                new ParallelCommandGroup(
                    cmd,
                    new SequentialCommandGroup(
                        new WaitCommand(profile.totalTime()),
                        new CreateAndDisown("elevator move", onArrivalCommandDealer)
                    )
                )
            );
    }

    private ezMotion getEzMotion(
        ezControl controller,
        TrapezoidProfile profile,
        TrapezoidProfile.State current,
        TrapezoidProfile.State goal
    ) {
        return new ezMotion(
            controller,
            this::getDistance,
            (double volts) -> {
                SmartDashboard.putNumber("Elevator volts", volts);
                this.setVoltage(volts);
            },
            (double t) -> {
                TrapezoidProfile.State result = profile.calculate(t, current, goal);

                SmartDashboard.putNumber("deg setpoint", result.velocity);
                return new Pair<>(result.position, result.velocity);
            },
            this
        );
    }
}
