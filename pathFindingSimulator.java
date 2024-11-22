import java.util.*;

public class pathFindingSimulator {

    // Directions for 4-way movement (Up, Down, Left, Right)
    private static final int[] directions_X = { 0, 1, 0, -1 };
    private static final int[] directions_Y = { -1, 0, 1, 0 };

    // Grid Representation: 0 = Open, 1 = Obstacle
    private static final int rows = 10;
    private static final int cols = 10;
    private static final int[][] grid = new int[rows][cols];

    // Node class to store information about each grid cell
    static class Node implements Comparable<Node> {
        int x, y, cost, heuristic, totalCost;
        Node parent;

        public Node(int x, int y, int cost, int heuristic) {
            this.x = x;
            this.y = y;
            this.cost = cost;
            this.heuristic = heuristic;
            this.totalCost = cost + heuristic;
            this.parent = null;
        }

        // Compare nodes by their total cost (used in A* and Dijkstra)
        @Override
        public int compareTo(Node other) {
            return Integer.compare(this.totalCost, other.totalCost);
        }
    }

    // A* Algorithm Implementation
    public static List<Node> aStar(int startX, int startY, int goalX, int goalY) {
        PriorityQueue<Node> openSet = new PriorityQueue<>();
        boolean[][] closedSet = new boolean[rows][cols];
        Node[][] allNodes = new Node[rows][cols];

        // Initializing the start node
        Node startNode = new Node(startX, startY, 0, manhattanDistance(startX, startY, goalX, goalY));
        openSet.add(startNode);
        allNodes[startX][startY] = startNode;

        while (!openSet.isEmpty()) {
            Node currentNode = openSet.poll();

            if (currentNode.x == goalX && currentNode.y == goalY) {
                return reconstructPath(currentNode);
            }

            closedSet[currentNode.x][currentNode.y] = true;

            // Explore neighbors
            for (int i = 0; i < 4; i++) {
                int newX = currentNode.x + directions_X[i];
                int newY = currentNode.y + directions_Y[i];

                if (isValid(newX, newY) && grid[newX][newY] == 0 && !closedSet[newX][newY]) {
                    int newCost = currentNode.cost + 1;
                    int newHeuristic = manhattanDistance(newX, newY, goalX, goalY);

                    Node neighbor = new Node(newX, newY, newCost, newHeuristic);
                    neighbor.totalCost = newCost + newHeuristic;
                    neighbor.parent = currentNode;

                    openSet.add(neighbor);
                    allNodes[newX][newY] = neighbor;
                }
            }
        }

        return new ArrayList<>(); // Return empty list if no path is found
    }

    // Dijkstra’s Algorithm Implementation
    public static List<Node> dijkstra(int startX, int startY, int goalX, int goalY) {
        PriorityQueue<Node> openSet = new PriorityQueue<>();
        boolean[][] closedSet = new boolean[rows][cols];
        Node[][] allNodes = new Node[rows][cols];

        // Initializing the start node
        Node startNode = new Node(startX, startY, 0, 0);
        openSet.add(startNode);
        allNodes[startX][startY] = startNode;

        while (!openSet.isEmpty()) {
            Node currentNode = openSet.poll();

            if (currentNode.x == goalX && currentNode.y == goalY) {
                return reconstructPath(currentNode);
            }

            closedSet[currentNode.x][currentNode.y] = true;

            // Explore neighbors
            for (int i = 0; i < 4; i++) {
                int newX = currentNode.x + directions_X[i];
                int newY = currentNode.y + directions_Y[i];

                if (isValid(newX, newY) && grid[newX][newY] == 0 && !closedSet[newX][newY]) {
                    int newCost = currentNode.cost + 1;

                    Node neighbor = new Node(newX, newY, newCost, 0);
                    neighbor.totalCost = newCost;
                    neighbor.parent = currentNode;

                    openSet.add(neighbor);
                    allNodes[newX][newY] = neighbor;
                }
            }
        }

        return new ArrayList<>(); // Return empty list if no path is found
    }

    // Manhattan Distance Heuristic (used in A* to estimate distance)
    private static int manhattanDistance(int x1, int y1, int x2, int y2) {
        return Math.abs(x1 - x2) + Math.abs(y1 - y2);
    }

    // Reconstruct the path from the goal node to the start node
    private static List<Node> reconstructPath(Node currentNode) {
        List<Node> path = new ArrayList<>();
        while (currentNode != null) {
            path.add(currentNode);
            currentNode = currentNode.parent;
        }
        Collections.reverse(path); // Reverse to get the path from start to goal
        return path;
    }

    // Check if a grid cell is within bounds and not an obstacle
    private static boolean isValid(int x, int y) {
        return x >= 0 && y >= 0 && x < rows && y < cols;
    }

    // Main method to run the simulation
    public static void main(String[] args) {
        // Initializing grid with obstacles (1 represents obstacle, 0 is empty)
        for (int i = 0; i < rows; i++) {
            Arrays.fill(grid[i], 0);
        }

        // Add some obstacles
        grid[1][2] = 1;
        grid[2][2] = 1;
        grid[3][2] = 1;
        grid[6][5] = 1;
        grid[7][5] = 1;
        grid[7][6] = 1;

        int startX = 0, startY = 0;
        int goalX = 9, goalY = 9;

        // Run A* Algorithm
        List<Node> aStarPath = aStar(startX, startY, goalX, goalY);
        System.out.println("A* Path:");
        for (Node n : aStarPath) {
            System.out.println("(" + n.x + ", " + n.y + ")");
        }

        // Run Dijkstra's Algorithm
        List<Node> dijkstraPath = dijkstra(startX, startY, goalX, goalY);
        System.out.println("\nDijkstra's Path:");
        for (Node n : dijkstraPath) {
            System.out.println("(" + n.x + ", " + n.y + ")");
        }
    }
}
