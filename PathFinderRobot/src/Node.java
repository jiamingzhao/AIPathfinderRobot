import java.awt.Point;
import java.util.ArrayList;

// Used to hold random point info, needs point information
public class Node extends Point {
	
	// Used for holding node paths
	ArrayList<Node> initialPath = new ArrayList<Node>();
	ArrayList<Node> neighbor = new ArrayList<Node>();

	// Values to help 
	String value = "Not-yet-set";
	int child;
	int max_val = Integer.MAX_VALUE;
	int nextChild;
	
	// Each node has a parent
	Node parent;
	
	// Each node has a child
	Node nodeChild;

}