
import heapq
import math

# Coordinates in Gazebo's world frame (meters)
nodes = {
    "START": (0.0, -1.3),
    "A": (0.0, 0.0),
    "B": (-1.5, 0.0),
    "C": (-1.5, 1.0),
    "D": (0.0, 1.0),
    "E": (1.5, 1.0),
    "F": (1.5, 0.0),
    "P1": (-1.5, 1.5),
    "P2": (-1.5, -1.5),
    "P3": (1.5, 1.5),
}

# Roads connecting intersections and stations
roads = [
    ("START", "A"),
    ("A", "B"),
    ("A", "D"),
    ("B", "C"),
    ("B", "P2"),
    ("C", "D"),
    ("C", "P1"),
    ("D", "E"),
    ("E", "F"),
    ("F", "A"),
    ("E", "P3"),
]

def find_route(start, goal, blocked_roads=None):
    blocked_roads = blocked_roads or set()
    graph = {name: [] for name in nodes}

    for a, b in roads:
        if frozenset((a, b)) in blocked_roads:
            continue

        weight = math.dist(nodes[a], nodes[b])
        graph[a].append((b, weight))
        graph[b].append((a, weight))

    queue = [(0.0, start, [start])]
    best = {}

    while queue:
        distance, current, path = heapq.heappop(queue)

        if current in best and best[current] <= distance:
            continue

        best[current] = distance

        if current == goal:
            return path

        for neighbor, weight in graph[current]:
            heapq.heappush(
                queue,
                (distance + weight, neighbor,
                 path + [neighbor])
            )

    return None


if __name__ == "__main__":
    current = "START"

    for station in ["P1", "P2", "P3"]:
        path = find_route(current, station)
        print(f"{current} -> {station}: {path}")
        current = station

    # Simulate a blocked road
    print("\nTesting road blockage:")
    blocked = {frozenset(("A", "B"))}
    print("A -> P2:",
          find_route("A", "P2", blocked))
