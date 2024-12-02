import heapq  # For implementing the priority queue in A* algorithm
import random  # For generating random obstacles, start, and goal positions


class AStar:
    def __init__(self, grid):
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, current):
        directions = [
            (0, 1),
            (1, 0),
            (0, -1),
            (-1, 0)
        ]

        neighbors = []
        for dx, dy in directions:
            new_x, new_y = current[0] + dx, current[1] + dy
            if (0 <= new_x < self.rows and
                0 <= new_y < self.cols and
                    self.grid[new_x][new_y] != 1):
                neighbors.append((new_x, new_y))
        return neighbors

    def find_path(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        closed_set = set()

        while open_set:
            current_f, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
            closed_set.add(current)
            for neighbor in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                tentative_g_score = g_score[current] + 1
                if neighbor not in [i[1] for i in open_set]:
                    heapq.heappush(open_set, (0, neighbor))
                elif tentative_g_score >= g_score.get(neighbor, float('inf')):
                    continue
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + \
                    self.heuristic(neighbor, goal)
        return None


def generate_random_grid(rows, cols, obstacle_count):
    grid = [[0 for _ in range(cols)] for _ in range(rows)]
    while obstacle_count > 0:
        x, y = random.randint(0, rows-1), random.randint(0, cols-1)
        if grid[x][y] == 0:
            grid[x][y] = 1
            obstacle_count -= 1
    start = (random.randint(0, rows-1), random.randint(0, cols-1))
    goal = (random.randint(0, rows-1), random.randint(0, cols-1))
    while goal == start or grid[goal[0]][goal[1]] == 1:
        goal = (random.randint(0, rows-1), random.randint(0, cols-1))
    grid[start[0]][start[1]] = 3
    grid[goal[0]][goal[1]] = 4
    return grid, start, goal


def main():
    rows, cols = 5, 5
    obstacle_count = 7
    grid, start, goal = generate_random_grid(rows, cols, obstacle_count)
    pathfinder = AStar(grid)
    path = pathfinder.find_path(start, goal)
    if path:
        print("Path found:", path)
        grid_copy = [row.copy() for row in grid]
        for x, y in path:
            if (x, y) == start:
                grid_copy[x][y] = 3
            elif (x, y) == goal:
                grid_copy[x][y] = 4
            elif (x, y) != goal:
                grid_copy[x][y] = 2

        # Update path_symbols with emojis
        path_symbols = {
            0: '⬜',   # Open path
            1: '🟥',   # Obstacle
            2: '🟡',   # Path
            3: '🟢',   # Start point
            4: '🔵'    # End point
        }
        for row in grid_copy:
            print(' '.join(path_symbols[cell] for cell in row))
    else:
        print("No path found!")


if __name__ == "__main__":
    main()
