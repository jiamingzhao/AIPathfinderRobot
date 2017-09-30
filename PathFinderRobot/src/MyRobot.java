/**
 * Jiaming Zhao
 * Pathfinder AI Work
 */

import world.Robot;
import world.World;

import java.awt.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.PriorityQueue;

import static java.lang.Math.abs;
import static java.lang.Math.min;
import static java.lang.Math.max;

public class MyRobot extends Robot {
    private final static double SQRT_2 = Math.sqrt(2);
    private static Point worldEndPosition;
    boolean isUncertain;

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

    /**
     * Get the heuristic distance cost based on diagonal steps needed to get
     * from Point p1 to Point p2 and on directSteps (up down left right)
     * @param p1
     * @param p2
     * @return the heuristic distance cost based on diagonal and direct steps
     */
    private double heuristicDistance(Point p1, Point p2) {
        double x1 = p1.getX(), x2 = p2.getX();
        double y1 = p1.getY(), y2 = p2.getY();

        double dx = abs(x2 - x1);
        double dy = abs(y2 - y1);

        double diagStepsDistance = min(dx, dy) * SQRT_2;
        double directStepsDistance = max(dx, dy) - min(dx, dy);

        return diagStepsDistance + directStepsDistance;
    }

    private ArrayList<Point> getNeighbors(Point myPosition) {
        int x = (int)(myPosition.getX()), y = (int)(myPosition.getY());
        ArrayList<Point> neighborsList = new ArrayList<Point>();
        int [] pts = {  // all 8 points
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

        System.out.println("Next possible nodes: ");
        for (int i = 0; i < pts.length; i+=2) {
            possiblePoint.setLocation(pts[i], pts[i+1]);
            // only add nodes that are "O" that can be moved to
            String ping = super.pingMap(possiblePoint);
            if (ping != null && ping.equals("O")) {
                System.out.println(possiblePoint.toString());
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
        /*
            get current position, x, and y
            priority queue with starting point for open nodes
            set with nothing (yet) that is for closed nodes
            while worst rank in open nodes is not goal
                current node = remove lowest rank from open nodes
                closed nodes += current node
                for the 8 neighbors available,
                cost = g(current) + cost(current, neighbor)
                if neighbor in open nodes and cost less than g(neighbor)
                    remove neighbor from open nodes (new path better)
                if neighbor not in open nodes and not in closed
                    set g(neighbor) to cost
                    add neighbor to open
                    set priority queue rank to g(neighbor) + h(neighbor)
                    set neighbor's parent to current
         */

        Point currentPosition = super.getPosition();

        PriorityQueue<ElementPriority> goNodes = new PriorityQueue<ElementPriority>();
        goNodes.add(new ElementPriority(currentPosition, 0));

        HashMap<Point, Point> previousNodes = new HashMap<Point, Point>();
        HashMap<Point, Double> cost = new HashMap<Point, Double>();

        previousNodes.put(currentPosition, currentPosition);
        cost.put(currentPosition, 0.0);

        while (!goNodes.isEmpty()) {
            ElementPriority currentPoint = goNodes.poll();

            // base case: reached end destination - stop traversing
            if (currentPoint.point.equals(worldEndPosition)) {
                break;
            }
            ArrayList<Point> neighbors = getNeighbors(currentPosition);

            for (Point point: neighbors) {

            }

        }
    }

    private void travelWithUncertaintyToDestination() {

    }

    @Override
    public void addToWorld(World world) {
        isUncertain = world.getUncertain();
        super.addToWorld(world);
    }

    public static void main(String[] args) {
        try {
            World myWorld = new World("TestCases/myInputFile1.txt", false);

            MyRobot robot = new MyRobot();
            robot.addToWorld(myWorld);
            myWorld.createGUI(400, 400, 200); // uncomment this and create a GUI; the last parameter is delay in msecs

            worldEndPosition = myWorld.getEndPos();
            robot.travelToDestination();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    class ElementPriority implements Comparable<ElementPriority> {
        Point point;
        int priority;

        public ElementPriority(Point point, int priority) {
            this.point = point;
            this.priority = priority;
        }

        @Override
        public int compareTo(ElementPriority otherElement) {
            return Integer.compare(this.priority, otherElement.priority);
        }
    }
}
