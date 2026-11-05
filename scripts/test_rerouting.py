from route_planner import find_route

blocked_roads = {
    frozenset(("A", "B"))
}

route = find_route(
    "A",
    "P1",
    blocked_roads
)

print("Blocked road: A -> B")
print("New route:", route)
