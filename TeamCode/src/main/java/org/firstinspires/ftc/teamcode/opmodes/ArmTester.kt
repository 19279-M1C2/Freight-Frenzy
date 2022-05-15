package org.firstinspires.ftc.teamcode.opmodes//package org.firstinspires.ftc.teamcode.opmodes
//
//import com.amarcolini.joos.command.RobotOpMode
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import org.firstinspires.ftc.teamcode.MainRobot
//import org.firstinspires.ftc.teamcode.util.telemetry.TelemetryUpdater
//
//@TeleOp(name = "Arm Tester", group = "Test")
//class ArmTester : RobotOpMode<MainRobot>() {
//    override fun preInit() {
//        initialize<MainRobot>()
//        robot.teleValues.add(TelemetryUpdater("leftTrig", robot.tele) {
//            "${robot.gamepad.p1.left_trigger.isActive}, ${robot.gamepad.p1.left_trigger.value}"
//        })
//    }
//
//    override fun preStart() {
//        robot.map(
//            { robot.gamepad.p1.left_trigger.isActive || robot.gamepad.p1.right_trigger.isActive },
//            { robot.spool.setPower(if (robot.gamepad.p1.left_trigger.isActive) 0.9 else if (robot.gamepad.p1.right_trigger.isActive) -.9 else 0.0) }
//        )
//    }
//
//}
//
//
//
