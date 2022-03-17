package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.Robot
import com.amarcolini.joos.command.RobotOpMode
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.Servo
import org.firstinspires.ftc.teamcode.components.arm.*


class MainRobot(private val opMode: RobotOpMode<MainRobot>) : Robot(opMode) {

    // declare your motors and sensors here. CRS, Servos, DC, Drivetrain
    lateinit var folds: Folds
    lateinit var mount: Mount
    lateinit var arm: Arm

    /**
     * [init] runs when the robot is in init.
     * use it to reset servo positions, make sure swerves are aligned, calibrate sensors
     * make sure to add telemetry too
     */
    override fun init() {
        folds = Folds(Servo(hMap, Constants.FIRST_FOLD_NAME), Servo(hMap, Constants.SECOND_FOLD_NAME))
        mount =
            Mount(
                MountSpin(
                    Motor(
                        hMap,
                        Constants.MOUNT_SPIN_NAME,
                        6000.0,
                        Constants.ULTRAPLANATRY_TICKS * Constants.MOUNT_SPIN_RATIO
                    )
                ),
                MountHeight(
                    Motor(
                        hMap,
                        Constants.MOUNT_HEIGHT_NAME,
                        6000.0,
                        Constants.ULTRAPLANATRY_TICKS * Constants.MOUNT_HEIGHT_RATIO
                    )
                )
            )

        arm = Arm(mount, folds)

        schedule(Command.of { mount.reset() }.requires(mount))
        schedule((folds.setSecondAngle(0.0) and folds.setFirstAngle(0.0)).requires(folds))
    }

    /**
     * This runs whenever the robot starts. It is automatically called by the teleOp
     */
    override fun start() {

    }
}