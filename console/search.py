import util

map = {0:[8,1], 1:[0,2], 2:[1,10], 3:[11,4], 4:[3,5], 5:[4,6], 6:[5], 7:[15], 8:[0,9,16], 9:[8], 10:[2,18],
       11:[3,19], 12:[13], 13:[12, 21, 4], 14:[13,15], 15:[7,14,23], 16:[8,17,24], 17:[16], 18:[10,19], 19:[11,18,20],
        20:[19,28], 21:[13,22], 22:[21], 23:[15,31], 24:[16, 25], 25:[24,26], 26:[25,27], 27:[26,35], 28:[20,29],
       29:[28,30], 30:[29,31], 31:[23,30], 32:[33,40], 33:[32,34], 34:[33,35], 35:[27,34,36], 36:[35,37], 37:[36],
       38:[39,46], 39:[38,47], 40:[32,48], 41:[32,49], 42:[41,43,50], 43:[42,44,51], 44:[43,45], 45:[44,46,53],
       46:[38,45], 47:[39,55], 48:[40,56], 49:[41,50], 50:[42,49], 51:[43,59], 52:[53], 53:[45,52,61], 54:[62],
       55:[47,63], 56:[48,57], 57:[56,58], 58:[57,59], 59:[51,58,60], 60:[59], 61:[53], 62:[54,63], 63:[55,62]}


def breadth_first_search(map, start, goal):
    path = []
    path.append(start)
    frontier = util.Queue()
    reached = list()
    reached.append(start)
    frontier.push(path)

    while not frontier.isEmpty():
        path_till_now = frontier.pop()
        current = path_till_now[-1]
        reached.append(current)
        if current == goal:
            return path_till_now, reached
        for next in map[current]:
            path_to_next = path_till_now.copy()
            path_to_next.append(next)
            if next not in reached:
                frontier.push(path_to_next)

def depth_first_search(map, start, goal):
    path = []
    path.append(start)
    frontier = util.Stack()
    reached = list()
    reached.append(start)
    frontier.push(path)

    while not frontier.isEmpty():
        path_till_now = frontier.pop()
        current = path_till_now[-1]
        reached.append(current)
        if current == goal:
            return path_till_now, reached
        for next in map[current]:
            path_to_next = path_till_now.copy()
            path_to_next.append(next)
            if next not in reached:
                frontier.push(path_to_next)


def uniform_cost_search(map, start, goal):
    path = []
    frontier = util.PriorityQueue()
    reached = list()

    if start == goal:
        return path, reached

    path.append(start)
    path_cost = 0

    frontier.push(path, path_cost)
    while not frontier.isEmpty():
        # pop a node from the queue
        path_cost_till_now, path_till_now = frontier.pop()
        current = path_till_now[-1]
        reached.append(current)

        # test goal condition
        if current == goal:
            return path_till_now, reached

        for next in map[current]:
            path_to_next = path_till_now.copy()
            path_to_next.append(next)

            extra_cost = 1
            next_cost = extra_cost + path_cost_till_now

            if (next not in reached):
                frontier.push(path_to_next, next_cost)
            elif next_cost < path_cost_till_now:
                frontier.pop()
                frontier.push(path_to_next, next_cost)


def astar_search(map, start, goal):

    path = []
    reached = list()
    frontier = util.PriorityQueue()

    if start == goal:
        return path, reached

    path.append(start)
    path_cost = util.manhattan_distance(start, goal)

    frontier.push(path, path_cost)
    while not frontier.isEmpty():
        # pop a node from the queue
        path_cost_till_now, path_till_now = frontier.pop()
        current = path_till_now[-1]
        path_cost_till_now = path_cost_till_now - util.manhattan_distance(current, goal)
        reached.append(current)

        if current == goal:
            return path_till_now, reached

        for next in map[current]:
            path_to_next = path_till_now.copy()
            path_to_next.append(next)

            extra_cost = 1
            next_cost = extra_cost + path_cost_till_now + util.manhattan_distance(next, goal)

            if next not in reached:
                frontier.push(path_to_next, next_cost)
            elif next_cost < path_cost_till_now :
                frontier.pop()
                frontier.push(path_to_next, next_cost)

path, reached = depth_first_search(map, 0, 61)
print("DFS: Path= ")
print(path)
print("Visited Nodes= ")
print(reached)
print()
path, reached = breadth_first_search(map, 0, 61)
print("BFS: Path= ")
print(path)
print("Visited Nodes= ")
print(reached)
print()
path, reached = uniform_cost_search(map, 0, 61)
print("UCS: Path= ")
print(path)
print("Visited Nodes= ")
print(reached)
print()
path, reached = astar_search(map, 0, 61)
print("AStar: Path= ")
print(path)
print("Visited Nodes= ")
print(reached)