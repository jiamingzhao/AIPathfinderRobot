
/**
 * Apoorva Arunkumar aa3vs
 * Jiaming Zhao jz4bm
 * Pathfinder AI Work
 * 
 * THANKS FOR GRADING THIS
 * 
 */

import world.Robot;
import world.World;

import java.awt.*;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

import static java.lang.Math.abs;
import static java.lang.Math.min;

public class MyRobot extends Robot {
	private final static double SQRT_2 = Math.sqrt(2);
	private static int numCols, numRows;
	private static Point worldEndPosition;
	HashSet<Point> closedNodes;
	boolean isUncertain;
	static String input;
	static World myWorld;

	// For uncertainty I made my own vars
	public static ArrayList<Node> seenList = new ArrayList<Node>();
	public static ArrayList<Node> unneededNodeList = new ArrayList<Node>();
	public static ArrayList<Node> traversalList = new ArrayList<Node>();
	public static ArrayList<Node> Walls = new ArrayList<Node>();

	// Start/finish nodes
	public static Node start;
	public static Node end;

	/**
	 * Get the heuristic distance cost based on direct steps needed to get from
	 * Point p1 to Point p2
	 * 
	 * @param p1
	 * @param p2
	 * @return the heuristic distance cost based on and direct steps
	 */
	private double heuristicDistance(Point p1, Point p2) {
		double x1 = p1.getX(), x2 = p2.getX();
		double y1 = p1.getY(), y2 = p2.getY();

		double dx = abs(x2 - x1);
		double dy = abs(y2 - y1);

		// number of steps without diagonal minus the difference when making diagonal
		// step
		// (2 - SQRT_2) represents replacing 2 regular steps minus the distance of a
		// diagonal step
		return (dx + dy) - (2 - SQRT_2) * min(dx, dy);
	}
	
	private void setUpWalls() {
		// TODO Auto-generated method stub
		// Set neighbor nodes
		for (int i = 0; i < Walls.size(); i++) {
			for (int j = 0; j < Walls.size(); j++) {
				// Iterate through and set neighbors of walls
				if ((Math.abs(Walls.get(i).x - Walls.get(j).x) == 1 || Walls.get(i).x == Walls.get(j).x)
						&& (Math.abs(Walls.get(i).y - Walls.get(j).y) == 1 || Walls.get(i).y == Walls.get(j).y)) {
					Walls.get(i).neighbor.add(Walls.get(j));
				}
			}
		}

	}

	// This is a method to basically check the certainty of a node.
	public void pingMapWithCertainty(Node x) {
		int count = 0;
		int distance = Math.abs(x.x - start.x) + Math.abs(x.y - start.y);
		distance *= distance;

		// This is arbitrary -- maybe changing it will help with pings lol
		// The distance is how far away I want to ping for this -- any longer will
		// probably not work
		if (distance < 12) {
			String a = this.pingMap(x);
			String b = this.pingMap(x);
			String c = this.pingMap(x);
			//String d = this.pingMap(x);

			// Check certainty
			if (a.equals("X")) {
				if (b.equals("X") && c.equals("X")) {
					x.value = "unsure-x";
					if (!unneededNodeList.contains(x)) {
						unneededNodeList.add(x);
					}
				}
				// Have no idea what this node is
				else
					x.value = "Destroy";
			}
			if (a.equals("O")) {
				if (b.equals("O") && c.equals("O")) {
					x.value = "unsure-o";
				}
				// Pinging did not work again
				else
					x.value = "Destroy";
			} else
				x.value = "Destroy";
		}

		else {
			for (int i = 0; i < distance; i++) {
				if (this.pingMap(x).equals("X")) {
					count++;
				}
			}
			// Zach used 7. Changing this may help pinging may not
			if (count > 6 * distance / 10) {
				x.value = "unsure-x";
				if (!unneededNodeList.contains(x)) {
					unneededNodeList.add(x);
				}
			} else if (count > 4 * distance / 10) {
				x.value = "Destroy";
			} else {
				x.value = "unsure-o";
			}
		}
	}

	/**
	 * Get the possible next Nodes to explore
	 * 
	 * @param myPosition
	 *            current Node's location
	 * @return next Nodes to explore (only the Nodes marked with "O")
	 */
	private ArrayList<Point> getNeighbors(Point myPosition) {
		int x = (int) (myPosition.getX()), y = (int) (myPosition.getY());
		ArrayList<Point> neighborsList = new ArrayList<Point>();
		int[] pts = { // 4 regular steps, 4 diagonal steps
				x - 1, y - 1, x - 1, y, x - 1, y + 1, x, y - 1, x, y + 1, x + 1, y - 1, x + 1, y, x + 1, y + 1 };

		Point possiblePoint = new Point(0, 0);

		// TODO maybe try to decrease number of pings somehow
		for (int i = 0; i < pts.length; i += 2) {
			if (pts[i] < 0 || pts[i + 1] < 0 || pts[i] >= numRows || pts[i + 1] >= numCols)
				continue;
			possiblePoint.setLocation(pts[i], pts[i + 1]);

			if (closedNodes.contains(possiblePoint))
				continue;

			// only add Nodes that are "O" or "F" that can be moved to
			String ping = super.pingMap(possiblePoint);
			if (ping != null) {
				if (ping.equals("O") || ping.equals("F"))
					neighborsList.add(possiblePoint.getLocation());
				else if (ping.equals("X"))
					closedNodes.add(possiblePoint.getLocation());
			}

		}

		return neighborsList;
	}

	/**
	 * Scan a large path before moving to decrease number of moves Scan more than
	 * move since all paths are certain and all spaces are seen Combine both Greedy
	 * Best + Dijkstra's = A* algo f = g + h where h is heuristic function
	 */
	private void travelWithCertaintyToDestination() {
		closedNodes = new HashSet<>();

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

			for (Point neighborPoint : neighbors) {
				double cost = previousCosts.get(currentPoint) + heuristicDistance(currentPoint, neighborPoint);

				if (!previousCosts.containsKey(neighborPoint) || cost < previousCosts.get(neighborPoint)) {
					previousCosts.put(neighborPoint, cost);
					double priority = cost + heuristicDistance(neighborPoint, worldEndPosition);

					goNodes.add(new ElementPriority(neighborPoint, priority));

					previousNodes.put(neighborPoint, currentPoint);
					closedNodes.add(currentPoint);
				}
			}
		}

		// generate the path from previousNodes
		ArrayList<Point> robotPath = new ArrayList<Point>();
		robotPath.add(worldEndPosition);

		Point point = previousNodes.get(worldEndPosition);

		if (point == null) {
			System.out.println("Safe passage not found. Robot will now shut down.");
			return;
		}

		while (!point.equals(startingPosition)) {
			robotPath.add(point);
			point = previousNodes.get(point);
		}

		// reverse the path from goal and proceed
		for (int i = robotPath.size() - 1; i >= 0; i--) {
			super.move(robotPath.get(i));
		}

	}

	// Basically A* algorithm but we ping spots around us to check if its valid or
	// not multiple times
	// Ping the map within a certain distance of us, anything further is probably
	// not helpful.
	// Traverse the path using the nodes added to the list.
	private void travelWithUncertaintyToDestination() {
		Node current = start;
		unneededNodeList.add(current);
		while (!current.neighbor.contains(end)) {

			// Want to run this for each neighbor
			pingNeighbors(current);

			int limit = Integer.MAX_VALUE;
			for (int k = 0; k < seenList.size(); k++) {
				if (seenList.get(k).max_val <= limit) {
					limit = seenList.get(k).max_val;
				}
			}

			// Add nodes to the path to traverse -- check if it is open or not, and if we
			// have no idea what the
			// Node is then we will ignore it.
			for (int i = 0; i < seenList.size(); i++) {
				if (seenList.get(i).max_val == limit) {
					if (seenList.get(i).value.equals("Destroy")) {
						if (this.pingMap(seenList.get(i)).equals("X")) {
							seenList.get(i).value = "X";
							if (!unneededNodeList.contains(seenList.get(i))) {
								unneededNodeList.add(seenList.get(i));
							}
						} else {
							seenList.get(i).value = "O";
							if (!unneededNodeList.contains(seenList.get(i))) {
								unneededNodeList.add(seenList.get(i));
							}
							seenList.get(i).initialPath.add(current);
							current = seenList.get(i);

						}
						seenList.remove(i);
					} else {
						if (!unneededNodeList.contains(seenList.get(i))) {
							unneededNodeList.add(seenList.get(i));
						}
						seenList.get(i).initialPath.add(current);
						current = seenList.get(i);
						seenList.remove(i);

						break;
					}
				}
			}
		}
		
		// Add our path to a list and traverse it
		addNodesToList(current);
	
		// Move to the spot
		for (int i = traversalList.size() - 1; i > -1; i--) {
			this.move(traversalList.get(i));
		}
	}

	private void addNodesToList(Node current) {
		// TODO Auto-generated method stub
		end.parent = current;
		Node pathNode = end;
		while (pathNode != start) {
			traversalList.add(pathNode);
			pathNode = pathNode.parent;
		}
		
	}

	// Ping neighbors and check for uncertainty
	private void pingNeighbors(Node current) {
		// TODO Auto-generated method stub
		for (int j = 0; j < current.neighbor.size(); j++) {

			Node temp = current.neighbor.get(j);
			while (temp.value.equals("Not-yet-set") || temp.value.equals("Destroy")) {

				// Try and ping the map right here
				// Ping each spot 4 times and SEE WHAT HAPPENS YOLO
				this.pingMapWithCertainty(temp);

			}
			
			// Now that the node is not-yet-set or not unknown, we can set the child and parent up.
			setUpChildAndParent(temp, current);

		}

	}

	private void setUpChildAndParent(Node temp, Node current) {
		// TODO Auto-generated method stub
		if (temp.max_val > current.child + 1 + temp.nextChild && !unneededNodeList.contains(temp)
				&& !temp.value.equals("unsure-x") && !temp.value.equals("X")) {
			temp.child = current.child + 1;
			temp.max_val = temp.child + temp.nextChild;
			if (!seenList.contains(temp)) {
				seenList.add(temp);
			}
			temp.parent = current;
		}
		
	}

	// Stores all map information in Nodes -- maybe unnecessary
	// Iterates through each line and stores all the nodes
	public void getPreliminaryInfo(String input) {
		BufferedReader br = null;
		try {
			br = new BufferedReader(new FileReader(input));
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		String line;
		int x = 0;
		try {
			while ((line = br.readLine()) != null) {
				// Split each character
				String temp[] = line.split(" ");
				// Check the values
				for (int i = 0; i < temp.length; i++) {
					Node tempNode = new Node();
					tempNode.setLocation(x, i);
					// Set the start / finish Nodes
					// Start
					if (temp[i].equals("S")) {
						tempNode.child = 0;
						tempNode.value = "S";
						start = tempNode;
					}
					// End
					if (temp[i].equals("F")) {
						tempNode.value = "F";
						end = tempNode;
					}
					// Otherwise it was a wall
					Walls.add(tempNode);
				}
				x++;
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// Check the nodes around the end position and see if there are walls around it,
		// and set its neighbors
		int wall_x = myWorld.getEndPos().x;
		int wall_y = myWorld.getEndPos().y;
		for (int i = 0; i < Walls.size(); i++) {
			int node = Math.abs(Walls.get(i).x - wall_x) + Math.abs(Walls.get(i).y - wall_y);
			Walls.get(i).nextChild = node;
		}

		// Figure out where wall neighbors are
		setUpWalls();

	}

	

	/**
	 * This method is for pathfinding
	 */
	@Override
	public void travelToDestination() {
		// use super.pingMap(new Point(x, y)) to see a part of the map
		// use super.move(new Point(x, y)) to move to a part of the map
		if (isUncertain) {
			getPreliminaryInfo(input);
			travelWithUncertaintyToDestination(); // uncertain
		} else {
			travelWithCertaintyToDestination(); // certain
		}
	}

	@Override
	public void addToWorld(World world) {
		isUncertain = world.getUncertain();
		super.addToWorld(world);
	}

	public static void main(String[] args) {
		try {
			// Remove PathFinderRobot/ if you can't find the file
			input = "TestCases/myInputFile4.txt";
			myWorld = new World(input, true);

			MyRobot robot = new MyRobot();
			robot.addToWorld(myWorld);
			
			
			myWorld.createGUI(400, 400, 200); // uncomment this and create a GUI; the
			// last parameter is delay in msecs

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
