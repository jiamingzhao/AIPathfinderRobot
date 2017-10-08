/**
 * Apoorva Arunkumar aa3vs
 * Jiaming Zhao jz4bm
 * Pathfinder AI Work
 */

import world.Robot;
import world.World;

import java.awt.*;
import java.util.*;

import static java.lang.Math.abs;
import static java.lang.Math.min;

public class MyRobot extends Robot {
    private final static double SQRT_2 = Math.sqrt(2);
    private static int numCols, numRows;
    private static Point worldEndPosition;
    boolean isUncertain;

    /**
     * Get the heuristic distance cost based on direct steps needed to get
     * from Point p1 to Point p2
     * @param p1
     * @param p2
     * @return the heuristic distance cost based on and direct steps
     */
    private double heuristicDistance(Point p1, Point p2) {
        double x1 = p1.getX(), x2 = p2.getX();
        double y1 = p1.getY(), y2 = p2.getY();

        double dx = abs(x2 - x1);
        double dy = abs(y2 - y1);

        // number of steps without diagonal minus the difference when making diagonal step
        // (2 - SQRT_2) represents replacing 2 regular steps minus the distance of a diagonal step
        return (dx + dy) - (2 - SQRT_2) * min(dx, dy);
    }

    /**
     * Get the possible next nodes to explore
     * @param myPosition current node's location
     * @return next nodes to explore (only the nodes marked with "O")
     */
    private ArrayList<Point> getNeighbors(Point myPosition) {
        int x = (int)(myPosition.getX()), y = (int)(myPosition.getY());
        ArrayList<Point> neighborsList = new ArrayList<Point>();
        int [] pts = {  // 4 regular steps, 4 diagonal steps
                x-1, y-1,
                x-1, y,
                x-1, y+1,
                x, y-1,
                x, y+1,
                x+1, y-1,
                x+1, y,
                x+1, y+1
        };

        Point possiblePoint = new Point(0, 0);

        // TODO maybe try to decrease number of pings somehow
        for (int i = 0; i < pts.length; i+=2) {
            if (pts[i] < 0 || pts[i+1] < 0 || pts[i] >= numRows || pts[i+1] >= numCols)
                continue;
            possiblePoint.setLocation(pts[i], pts[i+1]);

            // only add nodes that are "O" or "F" that can be moved to
            String ping = super.pingMap(possiblePoint);
            if (ping != null && (ping.equals("O") || ping.equals("F"))) {
                neighborsList.add(possiblePoint.getLocation());
            }
        }

        return neighborsList;
    }

    /**
     * Scan a large path before moving to decrease number of moves
     * Scan more than move since all paths are certain and all spaces are seen
     * Combine both Greedy Best + Dijkstra's = A* algo
     * f = g + h where h is heuristic function
     */
    private void travelWithCertaintyToDestination() {
        Point startingPosition = super.getPosition();

        PriorityQueue<ElementPriority> goNodes = new PriorityQueue<ElementPriority>();
        goNodes.add(new ElementPriority(startingPosition, 0.0));

        HashMap<Point, Point> previousNodes = new HashMap<Point, Point>();
        HashMap<Point, Double> previousCosts = new HashMap<Point, Double>();

        previousNodes.put(startingPosition, startingPosition);
        previousCosts.put(startingPosition, 0.0);

        while (!goNodes.isEmpty()) {
            ElementPriority currentElement = goNodes.poll();
            Point currentPoint = currentElement.point;

            // base case: reached end destination - stop traversing
            if (currentPoint.equals(worldEndPosition)) {
                System.out.println("Found world end destination with certainty! Reversing path and starting engines!");
                break;
            }
            ArrayList<Point> neighbors = getNeighbors(currentPoint);

            for (Point neighborPoint: neighbors) {
                double cost = previousCosts.get(currentPoint) +
                        heuristicDistance(currentPoint, neighborPoint);

                if (!previousCosts.containsKey(neighborPoint) || cost < previousCosts.get(neighborPoint)) {
                    previousCosts.put(neighborPoint, cost);
                    double priority = cost + heuristicDistance(neighborPoint, worldEndPosition);

                    goNodes.add(new ElementPriority(neighborPoint, priority));

                    previousNodes.put(neighborPoint, currentPoint);
                }
            }
        }

        // generate the path from previousNodes
        ArrayList<Point> robotPath = new ArrayList<Point>();
        robotPath.add(worldEndPosition);
        Point point = previousNodes.get(worldEndPosition);
        while (!point.equals(startingPosition)) {
            robotPath.add(point);
            point = previousNodes.get(point);
        }

        // reverse the path from goal and proceed
        for (int i = robotPath.size()-1; i >= 0; i--) {
            System.out.println(robotPath.get(i).toString());
            super.move(robotPath.get(i));
        }

    }

    private void travelWithUncertaintyToDestination() {

    }

    /**
     * This method is for pathfinding
     */
    @Override
    public void travelToDestination() {
        // use super.pingMap(new Point(x, y)) to see a part of the map
        // use super.move(new Point(x, y)) to move to a part of the map
        if (isUncertain) {
            travelWithUncertaintyToDestination();  // uncertain
        } else {
            travelWithCertaintyToDestination();  // certain
        }
    }

    @Override
    public void addToWorld(World world) {
        isUncertain = world.getUncertain();
        super.addToWorld(world);
    }

    public static void main(String[] args) {
        try {
            World myWorld = new World("TestCases/myInputFile4.txt", false);

            MyRobot robot = new MyRobot();
            robot.addToWorld(myWorld);
            myWorld.createGUI(400, 400, 200); // uncomment this and create a GUI; the last parameter is delay in msecs

            worldEndPosition = myWorld.getEndPos();
            numCols = myWorld.numCols();
            numRows = myWorld.numRows();

            robot.travelToDestination();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    class ElementPriority implements Comparable<ElementPriority> {
        Point point;
        double priority;

        public ElementPriority(Point point, double priority) {
            this.point = point;
            this.priority = priority;
        }

        @Override
        public int compareTo(ElementPriority otherElement) {
            return Double.compare(this.priority, otherElement.priority);
        }
    }
}
