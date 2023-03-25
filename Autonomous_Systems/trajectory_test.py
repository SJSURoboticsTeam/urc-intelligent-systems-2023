import numpy as np
from queue import PriorityQueue


class Map():
    def __init__(self, size):
        self.size = size
        self.obstacles = []

    def add_obstacle(self, position, radius):
        self.obstacles.append((position, radius))

    def is_traversable(self, position):
        for obstacle in self.obstacles:
            if np.linalg.norm(position - obstacle[0]) <= obstacle[1]:
                return False
        return True


class Trajectory():
    
    def distance(self, a, b):
        a = np.array(a)
        b = np.array(b)
        return np.linalg.norm(a - b)

    def generate_trajectory(self, start, goal, map):
        queue = PriorityQueue()
        queue.put((0, start))
        visited = {tuple(start): None}
        g_score = {tuple(start): 0}
        max_queue_size = 9
        
        while not queue.empty():
            if queue.qsize() >= max_queue_size:
                break  # queue has grown too large, algorithm failed
            print("Queue:", queue.queue) # Print the current state of the priority queue
            current = queue.get()[1]
            print("Current:", current) # Print the current node being evaluated
            if np.allclose(current, goal):
                break
            
            for direction in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)]:
                successor = (current[0] + direction[0], current[1] + direction[1])
                if not map.is_traversable(successor) or successor in visited:
                    continue
                
                tentative_g_score = g_score[tuple(current)] + self.distance(current, successor)
                if tuple(successor) not in g_score or tentative_g_score < g_score[tuple(successor)]:
                    g_score[tuple(successor)] = tentative_g_score
                    f_score = tentative_g_score + self.distance(successor, goal)
                    queue.put((f_score, successor))
                    visited[tuple(successor)] = tuple(current)
                    
        current = tuple(goal)
        trajectory = [np.array(current)]
        while current != tuple(start):
            if tuple(current) in visited:
             current = visited[current]
            else:
                break
            trajectory.append(np.array(current))
        trajectory.reverse()
        trajectory = np.array(trajectory)
        
        return trajectory



def test_generate_trajectory():
    # Generate random start and goal positions
    start = np.random.uniform(-10, 10, size=2)
    goal = np.random.uniform(-10, 10, size=2)

    # Create an instance of the Map class
    test_map = Map(size=20)

    # Add obstacles to the map
    test_map.add_obstacle(position=np.array([0, 0]), radius=5)
    test_map.add_obstacle(position=np.array([-5, -5]), radius=2)

    # Create an instance of the Trajectory class
    tg = Trajectory()

    # Call the function
    trajectory = tg.generate_trajectory(start, goal, test_map)

    # Check the output
    assert isinstance(trajectory, np.ndarray)
    #assert trajectory.shape == (101, 2)
    assert np.all(np.isfinite(trajectory))

    print("Test passed: trajectory generated successfully!")
    print("Start position: ", start)
    print("Goal position: ", goal)
    print("Trajectory: ", trajectory)


test_generate_trajectory()
