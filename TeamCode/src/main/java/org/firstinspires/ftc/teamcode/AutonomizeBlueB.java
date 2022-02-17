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


@Autonomous(name="Autonomous 2022 Blue B")
public class AutonomizeBlueB extends LinearOpMode {
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

        Pose2d startPose = new Pose2d(18, 56, Math.toRadians(270));

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
                .lineTo(new Vector2d(-20,19))
                // Drop cargo
                .addTemporalMarker(() -> grabber.setPower(-.1))
                .waitSeconds(.5)
                .addTemporalMarker(() -> grabber.setPower(0))
                // Goto warehouse
                .resetVelConstraint()
                .lineTo(new Vector2d(15,64))
                .lineTo(new Vector2d(57,64))
                .turn(Math.toRadians(90))
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