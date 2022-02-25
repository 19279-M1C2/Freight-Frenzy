package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.Robot
import com.amarcolini.joos.command.RobotOpMode
import com.amarcolini.joos.hardware.Motor
import com.amarcolini.joos.hardware.Servo
import org.firstinspires.ftc.teamcode.components.Arm
import org.firstinspires.ftc.teamcode.components.Folds
import org.firstinspires.ftc.teamcode.components.Mount


class MainRobot(opMode: RobotOpMode<MainRobot>, private val mode: OpMode) : Robot(opMode) {

    // we can create different op modes using this
    enum class OpMode {
        TeleOp,
        Auto,
    }

    // declare your motors and sensors here. CRS, Servos, DC, Drivetrain
    private lateinit var folds: Folds
    private lateinit var mount: Mount
    private lateinit var arm: Arm

    /**
     * [init] runs when the robot is in init.
     * use it to reset servo positions, make sure swerves are aligned, calibrate sensors
     * make sure to add telemetry too
     */
    override fun init() {
        folds = Folds(Servo(hMap, Constants.FIRST_FOLD_NAME), Servo(hMap, Constants.SECOND_FOLD_NAME))
        mount =
            Mount(
                Motor(
                    hMap,
                    Constants.MOUNT_SPIN_NAME,
                    6000.0,
                    Constants.ULTRAPLANATRY_TICKS * Constants.MOUNT_SPIN_RATIO
                ),
                Motor(
                    hMap,
                    Constants.MOUNT_HEIGHT_NAME,
                    6000.0,
                    Constants.ULTRAPLANATRY_TICKS * Constants.MOUNT_HEIGHT_RATIO
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
        // this is used to tell if we are running auto or teleop or other Op modes.
        if (mode == OpMode.TeleOp) {
            // we could also just call other functions here and split code
            TODO("TeleOp")
        } else if (mode == OpMode.Auto) {
            Command.of {
                folds.setFirstAngle(180.0)
                folds.setSecondAngle(180.0)
                mount.Height().setAngle(180.0)
            }.requires(
                folds, mount
            )


        }

    }
}