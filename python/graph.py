import numpy as np
from gtsam import *
from gpmp2 import *

from random import seed
from random import random


import numpy as np
from gtsam import * 
from gpmp2 import *
import matplotlib.pyplot as plt
from gpmp_utils.generate2Ddataset import generate2Ddataset
from gpmp_utils.signedDistanceField2D import signedDistanceField2D
from gpmp_utils.plotEvidenceMap2D import plotEvidenceMap2D
from gpmp_utils.plotPointRobot2D import plotPointRobot2D
from gpmp_utils.plotSignedDistanceField2D import plotSignedDistanceField2D
import copy

from pyrobot import Robot

dataset = generate2Ddataset('MultiObstacleDataset')
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y)

# Signed Distance field
field = signedDistanceField2D(dataset.map, cell_size)
sdf = PlanarSDF(origin_point2, cell_size, field)


# settings
total_time_sec = 10.0
total_time_step = 20
total_check_step = 50.0
delta_t = total_time_sec / total_time_step
check_inter = int(total_check_step / total_time_step - 1)

use_GP_inter = True


# point robot model
pR = PointRobot(2,1)
spheres_data = np.asarray([0.0,  0.0,  0.0,  0.0,  1.5])
nr_body = spheres_data.shape[0]
sphere_vec = BodySphereVector()
sphere_vec.push_back(BodySphere(spheres_data[0], spheres_data[4], \
        Point3(spheres_data[1:4])))
pR_model = PointRobotModel(pR, sphere_vec)

# GP
Qc = np.identity(2)
Qc_model = noiseModel_Gaussian.Covariance(Qc)

# Obstacle avoid settings
cost_sigma = 0.5
epsilon_dist = 4.0

# prior to start/goal
pose_fix = noiseModel_Isotropic.Sigma(2, 0.0001)
vel_fix = noiseModel_Isotropic.Sigma(2, 0.0001)


# start and end conf
start_conf = np.asarray([0, 0])
start_vel = np.asarray([0, 0])
end_conf = np.asarray([17, 14])
end_vel = np.asarray([0, 0])
avg_vel = (end_conf / total_time_step) / delta_t


# plot param
pause_time = total_time_sec / total_time_step




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
  result = optimizer.values()

  pos_keys = np.asarray(pos_keys)
  # TODO: check with mustafa if to include start and goal
  pos_key_vect = createKeyVector(pos_keys) 
  marginals = Marginals(graph, result)
  joint_marginal = marginals.jointMarginalCovariance(pos_key_vect)
  cov_mat = joint_marginal.fullMatrix()
  mean_chain = np.asarray(mean_chain)
  mean_chain = mean_chain.flatten()
  samples = np.random.multivariate_normal(mean_chain, cov_mat, nr_chains)

  initializations = []
  # fix start and goal state of the samples and also resize them properly!
  for i in range(nr_chains):
    initializations.append(samples[i,:].reshape((total_time_step+1, 
                                                  start_conf.shape[0])))
    initializations[-1][0,:] = start_conf
    initializations[-1][-1,:] = goal_conf

  return initializations


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

    
  def add_neighbour(self, node_key):
    if node_key in self.neighbours:
      print("The specified node is already a neighbour")
      return
    self.neighbours[node_key] = []

  def remove_neighbour(self, node_key):
    if node_key not in self.neighbours:
        print("The specified node is not a neighbour")
        return 
    del self.neighbours[node_key]



def get_planner_graph(inits, dropout_prob, avg_vel, seed_val=None):
  

  # we have total_time_step+1 points in each trajectory
  total_time_step = inits[0].shape[0] -1
  nr_chains = len(inits)

  # if nr_chains==1:
  #   print("Single chain passed. Please pass multiple chains")
  #   return

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

  for i in range(nr_chains): # go through each chain
    for j in range(0, total_time_step): # go through each time point
      # connect to start node
      if j == 0:
        first_idx = map_[(0,0)]
        second_idx = map_[(i,j+1)]
      # connect to goal node 
      elif j == total_time_step-1:
        first_idx = map_[(i,j)]
        second_idx = map_[(0,total_time_step)]
      else:
        first_idx = map_[(i,j)]
        second_idx = map_[(i,j+1)]        
      nodes[first_idx].add_neighbour(second_idx)

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
          first_idx = map_[(i,j)]
          second_idx = map_[(k,j+1)]
          nodes[first_idx].add_neighbour(second_idx)          

  return nodes


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
      graph.push_back(PriorFactorVector(key_vel, end_vel, vel_fix))
    
    if i > 0:
      #% cost factor
      graph.push_back(ObstaclePlanarSDFFactorPointRobot(key_pos, pR_model, 
                              sdf, cost_sigma, epsilon_dist))
      
      node_list[i].gt_graph_ob_id = graph.size()-1

    # add edges for each node

    for neigh_id in node_list[i].neighbours:
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
  return graph, init_values



#### Dijkstra specific stuff

from Queue import PriorityQueue

class Planner(object):
  """docstring for Graph"""
  def __init__(self, result, gtsam_graph, planner_graph):
    self.result = result
    self.gtsam_graph = gtsam_graph
    self.planner_graph = planner_graph

  def get_factor_error(self, gt_factor_id):
    
    return self.gtsam_graph.at(gt_factor_id).error(self.result)


  def get_edge_cost(self, first_idx, second_idx):
    cost = 0
    # add cost of gp and obstacle interpolation factors
    for gt_factor_id in self.planner_graph[first_idx].neighbours[second_idx]:
      cost += self.get_factor_error(gt_factor_id)
    # add cost of state obstacle factor
    if second_idx !=1:
      cost += self.get_factor_error(self.planner_graph[second_idx].gt_graph_ob_id)
    return cost

  def get_shortest_path(self):

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
    if cur_id !=1: 
      print("Failed to find a path. Check if everything is right")

    path.append(self.planner_graph[cur_id].pose)
    while cur_id !=0:
      cur_id = self.planner_graph[cur_id].parent_id
      path.append(self.planner_graph[cur_id].pose)

    path.reverse()
    return path

def update_planner_graph(result, planner_graph):
  for i in range(len(planner_graph)):
    planner_graph[i].pose = result.atVector(symbol(ord('x'), i))
    planner_graph[i].vel = result.atVector(symbol(ord('v'), i))

if __name__ == "__main__":
  inits = get_initializations(start_conf, end_conf, 4, total_time_step, avg_vel)
  print(inits)

  planner_graph = get_planner_graph(inits, dropout_prob=0.5, avg_vel=avg_vel, seed_val=1)
  print(len(planner_graph))

  gtsam_graph, init_values = get_gtsam_graph(planner_graph)

  print(gtsam_graph)

  use_trustregion_opt = True

  if use_trustregion_opt:
    parameters = DoglegParams()
    #parameters.setVerbosity('ERROR')
    optimizer = DoglegOptimizer(gtsam_graph, init_values, parameters)
  else:
    parameters = GaussNewtonParams()
    #parameters.setRelativeErrorTol(1e-5)
    #parameters.setMaxIterations(100)
    #parameters.setVerbosity('ERROR')
    optimizer = GaussNewtonOptimizer(gtsam_graph, init_values, parameters)

  print('Initial Error = %d\n', gtsam_graph.error(init_values))


  optimizer.optimizeSafely()
  result = optimizer.values()

  print('Final Error = %d\n', gtsam_graph.error(result))

  update_planner_graph(result, planner_graph)

  planner = Planner(result, gtsam_graph, planner_graph)
  path = planner.get_shortest_path()
  print(path)


## plot final values
figure = plt.figure()
axis = figure.gca()
# plot world
plotEvidenceMap2D(figure, axis, dataset.map, dataset.origin_x, dataset.origin_y, cell_size)
for i in range(total_time_step+1):
    axis.set_title('Optimized Values')
    # plot arm
    conf = path[i]
    #conf = result.atVector(symbol(ord('x'), i))
    plotPointRobot2D(figure, axis, pR_model, conf)
    plt.pause(pause_time)



