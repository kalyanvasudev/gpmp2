import numpy as np
from gtsam import *
from gpmp2 import *

from random import seed
from random import random


def get_initializations(start_conf, goal_conf, nr_chains, total_time_step, avg_vel):
  mean_chain = []
  pos_keys = []
  # init optimization
  graph = NonlinearFactorGraph()
  init_values = Values()


  for i in range(0, total_time_step+1):
    key_pos = symbol(ord('x'), i)
    pos_keys.append(key_pos) 
    key_vel = symbol(ord('v'), i)
    
    # initialize as straight line in conf space
    pose = start_conf * float(total_time_step-i)/float(total_time_step) +\
                                         end_conf * i/float(total_time_step)
    mean_chain.append(pose)
    vel = avg_vel
    print(pose)
    init_values.insert(key_pos, pose)
    init_values.insert(key_vel, vel)
    
    # start/end priors
    if i==0:
      graph.push_back(PriorFactorVector(key_pos, start_conf, pose_fix))
      graph.push_back(PriorFactorVector(key_vel, start_vel, vel_fix))
    elif i==total_time_step:
      graph.push_back(PriorFactorVector(key_pos, end_conf, pose_fix))
      graph.push_back(PriorFactorVector(key_vel, end_vel, vel_fix))

    # GP priors and cost factor
    if i > 0:
      key_pos1 = symbol(ord('x'), i-1)
      key_pos2 = symbol(ord('x'), i)
      key_vel1 = symbol(ord('v'), i-1)
      key_vel2 = symbol(ord('v'), i)

      temp = GaussianProcessPriorLinear(key_pos1, key_vel1,
                key_pos2, key_vel2, delta_t, Qc_model)
      graph.push_back(temp)

  parameters = GaussNewtonParams()
  optimizer = GaussNewtonOptimizer(graph, init_values, parameters)
  pos_keys = np.asarray(pos_keys)
  # TODO: check with mustafa if to include start and goal
  pos_key_vect = createKeyVector(pos_keys) 
  joint_marginal = marginals.jointMarginalCovariance(key_vect)
  cov_mat = joint_marginal.fullMatrix()
  mean_chain = np.asarray(mean_chain)

  

  samples = np.random.multivariate_normal(mean_chain, cov_mat, nr_chains)

  initializations = []
  # fix start and goal state of the samples and also resize them properly!
  for i in range(nr_chains):
    initializations.append(samples[i,:].reshape((total_time_step+1, 
                                                  start_conf.shape[0])))
    initializations[-1][0,:] = start_conf
    initializations[-1][-1,:] = goal_conf

  return samples


class Node(object):
  """docstring for Node"""
  def __init__(self, planner_id, pose, vel=None):
    self.planner_id = planner_id
    self.pose = pose
    self.vel = vel

    # A* search specific things
    self.visited = None
    self.parent_id = None

    # gtsam factor graph related stuff
    self.gt_graph_ob_id = None # obstacle factor id at the node
    # key is planner_id and value is list of gtsam factor ids
    # here value list consists of gp factors and interpolation obstacle factors
    self.neighbours = {}

    
  def add_neighbour(node_key):
    if node_key in self.neighbours:
      print("The specified node is already a neighbour")
      return
    self.neighbours[node_key] = []

  def remove_neighbour(node_key):
    if node_key not in self.neighbours:
        print("The specified node is not a neighbour")
        return 
    del self.neighbours[node_key]



# class Planner(object):
#   """docstring for Graph"""
#   def __init__(self):
#     self.start_node_key = None

#   def get_shortest_path(start_node):
#     self.start_node_key = s

  
def get_planner_graph(inits, dropout_prob, avg_vel, seed_val=None):
  

  # we have total_time_step+1 points in each trajectory
  total_time_step = inits[0].shape[1] -1
  nr_chains = len(inits)

  if nr_chains==1:
    print("Single chain passed. Please pass multiple chains")
    return

  if total_time_step ==0:
    print("total_time_step cannot be 0")
    return
  
  # planner id is same as idx in nodes
  nodes = [] # contains all nodes.
  
  map_ = {} # key is (chain_number, timestamp)

  planner_id = 0
  map_[(0,0)] = 0 
  start_node = Node(planner_id, inits[0][0,:], avg_vel)

  planner_id += 1
  map_[(0,total_time_step)] = 1
  goal_node = Node(planner_id, inits[0][-1,:], avg_vel)

  nodes.append(start_node)
  nodes.append(goal_node)


  # create all nodes
  for i in range(nr_chains): # go through each chain
    for j in range(1,total_time_step): # go through each time point

      planner_id = len(nodes)
      nodes.append(Node(planner_id, inits[i][j,:], avg_vel))
      map_[(i,j)] = planner_id

  # add all segential inchain connections
  for i in nr_chains:

    # connect start to all chains
    1st_idx = map_[(0,0)] 
    2nd_idx = map_[(i,1)]
    nodes[1st_idx].add_neighbour(2nd_idx)

    # connect goal to all chains
    1st_idx = map_[(i,total_time_step-1)] 
    2nd_idx = map_[(0,total_time_step)]
    nodes[1st_idx].add_neighbour(2nd_idx)

  for i in range(nr_chains): # go through each chain
    for j in range(0, total_time_step): # go through each time point

      1st_idx = map_[(i,j)]
      2st_idx = map_[(i,j+1)]

      # connect to start node
      if j == 0:
        1st_idx = map_[(0,0)]

      # connect to goal node 
      elif j == total_time_step-1:
        2nd_idx = map_[(0,total_time_step)]

      nodes[1st_idx].add_neighbour(2nd_idx)

  # add random inter connections
  if seed_val is not None:
    seed(seed_val)

  for i in range(nr_chains): # go through each chain
    for j in range(1,total_time_step-1): # go through each time point
      for k in range(nr_chains): # choose a branch to connect to
        if i == k:
          break

        rand_num = random()
        if random < dropout_prob:
          1st_idx = map_[(i,j)]
          2nd_idx = map_[(k,j+1)]
          nodes[1st_idx].add_neighbour(2nd_idx)          

  return nodes, map_


def get_gtsam_graph(node_list):


  # init optimization
  graph = NonlinearFactorGraph()
  init_values = Values()

  # add all nodes
  for i in range(len(node_list)):
    key_pos = symbol(ord('x'), i) 
    key_vel = symbol(ord('v'), i)
    
    #% initialize as straight line in conf space
    init_values.insert(key_pos, node_list[i].pose)
    init_values.insert(key_vel, node_list[i].vel)
    
    #% start/end priors
    if i==0:
      graph.push_back(PriorFactorVector(key_pos, node_list[i].pose, pose_fix))
      graph.push_back(PriorFactorVector(key_vel, start_vel, vel_fix))
    elif i==1:
      graph.push_back(PriorFactorVector(key_pos, node_list[i].pose, pose_fix))
      graph.push_back(PriorFactorVector(key_vel, goal_vel, vel_fix))

    # GP priors and cost factor
    if i > 0:
      #% cost factor
      graph.push_back(ObstaclePlanarSDFFactorPointRobot(key_pos, pR_model, 
                              sdf, cost_sigma, epsilon_dist))
      
      node_list[i].gt_graph_ob_id = graph.size()-1

    # add edges for each node

    for neigh_id in node[i].neighbours:
      key_pos1 = symbol(ord('x'), i)
      key_pos2 = symbol(ord('x'), neigh_id)
      key_vel1 = symbol(ord('v'), i)
      key_vel2 = symbol(ord('v'), neigh_id)

      graph.push_back(GaussianProcessPriorLinear(key_pos1, key_vel1,
            key_pos2, key_vel2, delta_t, Qc_model))
      node_list[i].neighbours[neigh_id].append(graph.size()-1)

      #% GP cost factor
      if use_GP_inter and check_inter > 0:
        for j in range(1, check_inter+1):
          tau = j * (total_time_sec / total_check_step)
          graph.add(ObstaclePlanarSDFFactorGPPointRobot(
                key_pos1, key_vel1, key_pos2, key_vel2,
                pR_model, sdf, cost_sigma, epsilon_dist,
                Qc_model, delta_t, tau))
          node_list[i].neighbours[neigh_id].append(graph.size()-1)          




#### Dijkstra specific stuff

from queue import PriorityQueue

class Planner(object):
  """docstring for Graph"""
  def __init__(self, result, gtsam_graph, planner_graph):
    self.result = result
    self.gtsam_graph = gtsam_graph
    self.planner_graph = planner_graph

  def get_factor_error(gt_factor_id):
    
    return self.gtsam_graph.at(gt_factor_id).error(self.result)


  def get_edge_cost(1st_idx, 2nd_idx):
    cost = 0
    # add cost of gp and obstacle interpolation factors
    for gt_factor_id in self.planner_graph[1st_idx].neighbours[2nd_idx]:
      cost += self.get_factor_error(gt_factor_id)
    # add cost of state obstacle factor
    cost += self.get_factor_error(self.planner_graph[2nd_idx].gt_graph_ob_id)
    return cost

  def get_shortest_path(gtsam_graph, planner_graph):

    cur_id = 0
    priority_q = PriorityQueue()
    priority_q.put((0, cur_id))
    self.planner_graph[cur_id].visited = True

    while priority_q.qsize() > 0:
      cur_cost, cur_id = priority_q.get()

      if cur_id == 1: # goal id 
        break

      for neigh_id in self.planner_graph[cur_id].neighbours:
        if self.planner_graph[neigh_id].visited == True:
          continue
        cost = self.get_edge_cost(cur_id, neigh_id)
        priority_q.put((cur_cost + cost, neigh_id))
        self.planner_graph[neigh_id].visited = True
        self.planner_graph[neigh_id].parent_id = cur_id

    path = []
    cur_id = 1
    path.append(self.planner_graph[cur_id].pose)
    while cur_id !=0:
      cur_id = self.planner_graph[cur_id].parent_id
      path.append(self.planner_graph[cur_id].pose)

    return path.reverse()



