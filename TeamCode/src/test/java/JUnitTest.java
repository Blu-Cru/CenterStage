import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.internal.system.Assert;

import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization.PoseHistory;
import org.firstinspires.ftc.teamcode.blucru.common.subsystems.drivetrain.localization.PoseMarker;
import org.junit.*;

public class JUnitTest {
    @Test
    public void testCar() {
        Assert.assertEquals(1, 0);
    }

    @Test
    public void testPoseHistory() {
        PoseHistory poseHistory = new PoseHistory();
        poseHistory.add(new Pose2d(0, 0, 0));
        long time1 = System.nanoTime();

        poseHistory.add(new Pose2d(1, 1, 1));
        long time2 = System.nanoTime();

        poseHistory.add(new Pose2d(2, 2, 2));
        long time3 = System.nanoTime();

        poseHistory.offset(new Pose2d(1, 1, 1));

        Assert.assertTrue(poseHistory.getPoseAtTime(time2).equals(new Pose2d(2, 2, 1)));
    }
}
