import heapq
import time

# Grid kota simulasi
city_map = [
    ['R', '.', '.', '.', '.', '.', '.', '.', '.', 'C'],
    ['.', 'X', '3', '.', 'X', '.', '.', 'X', '.', '.'],
    ['.', '.', '.', '.', '.', '.', '2', '.', '.', '.'],
    ['.', 'X', '.', 'X', '.', '4', '.', '.', '.', '.'],
    ['.', '.', '.', '.', '.', '.', '.', '.', '.', '.']
]

ROWS = len(city_map)
COLS = len(city_map[0])
DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]

# Fungsi mencari posisi simbol di grid
def find_position(symbol):
    for r in range(ROWS):
        for c in range(COLS):
            if city_map[r][c] == symbol:
                return (r, c)
    return None

# Heuristik Manhattan
def manhattan(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

# Biaya berdasarkan lalu lintas
def cell_cost(r, c):
    val = city_map[r][c]
    return int(val) if val.isdigit() else 1

# Fungsi pencarian jalur (bisa A* atau GBFS)
def search(grid, start, goal, method='astar'):
    frontier = []
    heapq.heappush(frontier, (0, start))
    came_from = {start: None}
    cost_so_far = {start: 0}
    explored_nodes = 0

    while frontier:
        _, current = heapq.heappop(frontier)
        explored_nodes += 1

        if current == goal:
            break

        for dr, dc in DIRECTIONS:
            nr, nc = current[0] + dr, current[1] + dc
            next_cell = (nr, nc)

            if 0 <= nr < ROWS and 0 <= nc < COLS and grid[nr][nc] != 'X':
                new_cost = cost_so_far[current] + cell_cost(nr, nc)

                if next_cell not in cost_so_far or new_cost < cost_so_far[next_cell]:
                    cost_so_far[next_cell] = new_cost
                    heuristic = manhattan(next_cell, goal)

                    if method == 'astar':
                        priority = new_cost + heuristic
                    elif method == 'gbfs':
                        priority = heuristic
                    else:
                        raise ValueError("Metode harus 'astar' atau 'gbfs'.")

                    heapq.heappush(frontier, (priority, next_cell))
                    came_from[next_cell] = current

    return reconstruct_path(came_from, start, goal), explored_nodes

# Rekonstruksi jalur dari hasil pencarian
def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        if current not in came_from:
            return []
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path

# ========================
# Eksekusi dan Perbandingan
# ========================
if __name__ == "__main__":
    start = find_position('R')
    goal = find_position('C')

    results = {}

    for algo in ['astar', 'gbfs']:
        t0 = time.time()
        path, explored = search(city_map, start, goal, method=algo)
        t1 = time.time()
        elapsed_ms = (t1 - t0) * 1000

        results[algo] = {
            "path": path,
            "time_ms": elapsed_ms,
            "nodes": explored
        }

        print(f"\nMetode: {algo.upper()}")
        print("Path:", path)
        print(f"Time: {elapsed_ms:.4f} ms")
        print(f"Jumlah node dieksplorasi: {explored}")

    # Ringkasan perbandingan
    print("\n📊 Perbandingan:")
    print(f"A*   -> Time: {results['astar']['time_ms']:.4f} ms, Nodes: {results['astar']['nodes']}")
    print(f"GBFS -> Time: {results['gbfs']['time_ms']:.4f} ms, Nodes: {results['gbfs']['nodes']}")
