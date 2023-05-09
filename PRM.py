import numpy as np
from sympy import Point3D, Segment, Plane


# obs [[x_pos,y_pos,z_pos,x_width,y_width,z_width],...]
def gen_PRM(num_nodes, obs):
    nodes = gen_nodes(num_nodes, obs)
    roadmap = add_paths(nodes, obs)
    return roadmap


def generate_path(start, goal, num_nodes, obs):
    roadmap = gen_PRM(num_nodes, obs)
    p1 = point_to_PRM(start, roadmap, obs)
    p2 = point_to_PRM(goal, roadmap, obs)
    if p1 == False or p2 == False:
        print('no path found')
        return []
    path = bfs(roadmap, p1, p2)
    if path == []:
        return []
    path.append(goal)
    return path


def bfs(roadmap, p1, p2):
    path = []
    explored = {}
    queue = [(p1,"Start")]
    while len(queue) > 0 and p2 not in roadmap[queue[0][0]]:
        cur = queue[0][0]
        explored[cur] = queue[0][1]
        children = roadmap[tuple(cur)]
        queue.remove(queue[0])
        for i in range(len(children)):
            if children[i] not in explored:
                queue.append((children[i], cur))
    if len(queue) == 0:
        return []
    path.append(p2)
    cur = queue[0][0]
    explored[cur] = queue[0][1]
    while explored[cur] != "Start":
        path.append(cur)
        cur = explored[cur]
    path.append(p1)
    path.reverse()
    return path


def point_to_PRM(p, roadmap, obs):
    p = np.array(p)
    nodes = np.array(list(roadmap.keys()))
    dists = np.sqrt(np.sum((p-nodes)**2,1))
    sorted_indices = np.argsort(dists)
    for i in range(len(sorted_indices)):
        if valid_path(p, nodes[sorted_indices[i]], obs):
            return tuple(nodes[sorted_indices[i]])
    return False


def gen_nodes(num_nodes, obs):
    found = 0
    nodes = np.ndarray((num_nodes, 3))
    while found < num_nodes:
        pos = np.random.rand(3) * 10 - 5
        if valid_pos(pos, obs):
            nodes[found] = pos
            found += 1
    return nodes

def add_paths(nodes, obs):
    roadmap = {}
    for i in range(len(nodes)):
        dists = np.sqrt(np.sum((nodes[i]-nodes)**2,1))
        roadmap[tuple(nodes[i])] = []
        sorted_indices = np.argsort(dists)
        lowest_indices = []
        j = 1
        if tuple(nodes[i]) not in list(roadmap.keys()):
            roadmap[tuple(nodes[i])] = []
        while len(lowest_indices) + len(roadmap[tuple(nodes[i])]) < 3:
            if valid_path(nodes[i], nodes[sorted_indices[j]], obs):
                lowest_indices.append(sorted_indices[j])
            j += 1
        if len(lowest_indices) > 0:
            closest = nodes[lowest_indices].tolist()
            for element in closest:
                if tuple(element) not in list(roadmap.keys()):
                    roadmap[tuple(element)] = []
                roadmap[tuple(element)].append(tuple(nodes[i]))
                roadmap[tuple(nodes[i])].append(tuple(element))
    return roadmap

def valid_pos(pos, obs):
    if pos[0] < 0.15 or pos[0] > 10 - 0.15:
        return False
    if pos[1] < 0.15 or pos[1] > 10 - 0.15:
        return False
    if pos[2] < 0.1:
        return False
    
    for o in obs:
        if pos[0] > o[0] - o[3]/2 and pos[0] < o[0] + o[3]/2:
            return False
        if pos[1] > o[1] - o[4]/2 and pos[1] < o[1] + o[4]/2:
            return False
        if pos[2] > o[2] - o[5]/2 and pos[2] < o[2] + o[5]/2:
            return False
        
    return True

def valid_path(q1, q2, obs):
    p1, p2 = Point3D(q1), Point3D(q2)
    segment = Segment(p1,p2)
    for o in obs:
        o_min = o[:3] - o[3:] / 2 - .15
        o_max = o[:3] + o[3:] / 2 + .15
        planes = np.array([
            Plane(Point3D(o_min[0], o_min[1], o_min[2]), normal_vector=(1, 0, 0)),
            Plane(Point3D(o_max[0], o_min[1], o_min[2]), normal_vector=(-1, 0, 0)),
            Plane(Point3D(o_min[0], o_max[1], o_min[2]), normal_vector=(0, 1, 0)),
            Plane(Point3D(o_min[0], o_min[1], o_max[2]), normal_vector=(0, 0, 1)),
            Plane(Point3D(o_max[0], o_max[1], o_min[2]), normal_vector=(0, -1, 0)),
            Plane(Point3D(o_max[0], o_min[1], o_max[2]), normal_vector=(0, 0, -1))
        ])
        for p in planes:
            intersect = p.intersection(segment)
            if len(intersect) != 0:
                for i in range(len(intersect)):
                    if (intersect[i][:] >= o_min).all() and (intersect[i][:] <= o_max).all():
                        return False
                break
    return True