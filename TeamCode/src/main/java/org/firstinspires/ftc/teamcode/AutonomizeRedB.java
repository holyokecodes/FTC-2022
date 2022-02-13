package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name="Autonomous 2022 Red B")

public class AutonomizeRedB extends LinearOpMode {
    SampleMecanumDrive drivetrain;

    private DcMotor carousel;
    private DcMotor arm;
    private CRServo grabber;

    @Override
    public void runOpMode() {
        drivetrain = new SampleMecanumDrive(hardwareMap);

        carousel = hardwareMap.get(DcMotor.class, "carousel");
        arm = hardwareMap.get(DcMotor.class, "arm");
        grabber = hardwareMap.get(CRServo.class, "grabber");

        Pose2d startPose = new Pose2d(15, -60, Math.toRadians(90));

        drivetrain.setPoseEstimate(startPose);

        //initialize arm
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        TrajectorySequence trajSeq = drivetrain.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> grabber.setPower(.1))
                // Raise arm
                .addTemporalMarker(() -> arm.setPower(-.1))
                .addTemporalMarker(() -> arm.setTargetPosition(-325))
                .waitSeconds(2)
                // Goto shipping hub
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineTo(new Vector2d(-6,-29))
                // Drop cargo
                .addTemporalMarker(() -> grabber.setPower(-.1))
                .waitSeconds(.5)
                .addTemporalMarker(() -> grabber.setPower(0))
                // Goto carousel
                .resetVelConstraint()
                .lineTo(new Vector2d(-70,-54))
                //.turn(Math.toRadians(-30))
                .UNSTABLE_addDisplacementMarkerOffset(0.5,() -> carousel.setPower(-.8))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .back(75)
                // Spin duck carousel
                //.waitSeconds(4)
                .addTemporalMarker(() -> carousel.setPower(0))
                .addTemporalMarker(() -> grabber.setPower(.1))
                .waitSeconds(.5)
                .addTemporalMarker(() -> grabber.setPower(0))
                .build();

        telemetry.addData(">", "Press Play to start op mode, amd wait for the movement of the robot.");
        telemetry.update();

        /** Wait for the game to begin */
        waitForStart();

        if (opModeIsActive()) {
            drivetrain.followTrajectorySequence(trajSeq);
        }
    }
}