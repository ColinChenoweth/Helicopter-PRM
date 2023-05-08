import numpy as np

# obs [[x_pos,y_pos,z_pos,x_width,y_width,z_width],...]
def generate_PRM(num_nodes, obs):
    nodes = gen_nodes(num_nodes, obs)
    roadmap = add_paths(nodes, obs)
    return roadmap

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
        while len(lowest_indices) < 3:
            if valid_path(nodes[i], nodes[sorted_indices[j]], obs):
                lowest_indices.append(sorted_indices[j])
        lowest_indices = sorted_indices[:4]
        roadmap[tuple(nodes[i])] = nodes[lowest_indices].tolist()
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

def valid_path(p1, p2, obs):
    return True

rm = generate_PRM(10, np.array([[0, 0, 5, 1.5, 1.5, 1.5]]))
print()