package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleopOpMode extends OpMode {

    private DriveSubsystem driveSubsystem;

    @Override
    public void init() {
        driveSubsystem = new DriveSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        driveSubsystem.resetYaw();
    }

    @Override
    public void loop() {
        // Read controller input
        double translationY = -gamepad1.left_stick_y;
        double translationX = gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;
        double reductionFactor = gamepad1.right_bumper ? 4.0 : 1.0;

        // Square forward, strafe, and turn inputs while keeping their sign
        translationY *= Math.abs(translationY);
        translationX *= Math.abs(translationX);
        rotation *= Math.abs(rotation);

        driveSubsystem.drive(translationY, translationX, rotation, reductionFactor);

        if (gamepad1.back) {
            driveSubsystem.resetYaw();
        }
    }

    @Override
    public void stop() {
        driveSubsystem.stop();
    }
}