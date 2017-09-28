/**
 * Jiaming Zhao
 * Pathfinder AI Work
 */

import world.Robot;
import world.World;

import java.awt.*;
import java.util.*;

public class MyRobot extends Robot {
    boolean isUncertain;

    /**
     * This method is for pathfinding
     */
    @Override
    public void travelToDestination() {
        // use super.pingMap(new Point(x, y)) to see a part of the map
        // use super.move(new Point(x, y)) to move to a part of the map
        if (isUncertain) {
            // call function to deal with uncertainty
        } else {
            // call function to deal with certainty
        }
    }

    @Override
    public void addToWorld(World world) {
        isUncertain = world.getUncertain();
        super.addToWorld(world);
    }

    public static void main(String[] args) {
        try {
            World myWorld = new World("TestCases/myInputFile1.txt", true);

            MyRobot robot = new MyRobot();
            robot.addToWorld(myWorld);
            myWorld.createGUI(400, 400, 200); // uncomment this and create a GUI; the last parameter is delay in msecs


            robot.travelToDestination();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
