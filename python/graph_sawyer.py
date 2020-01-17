import numpy as np
from gtsam import * 
from gpmp2 import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D #<-- Note the capitalization! 
from gpmp_utils.generate3Ddataset import generate3Ddataset
from gpmp_utils.signedDistanceField3D import signedDistanceField3D
from gpmp_utils.generateArm import generateArm
from gpmp_utils.plotMap3D import plotMap3D
from gpmp_utils.plotRobotModel import plotRobotModel
from gpmp_utils.set3DPlotRange import set3DPlotRange
from gpmp_utils.plotArm import plotArm
from pyrobot import Robot
# dataset
from random import seed
from random import random


import copy
from pyrobot import Robot

# dataset
dataset = generate3Ddataset('WAMDeskDataset')
origin = np.asarray([dataset.origin_x, dataset.origin_y, dataset.origin_z])
origin_point3 = Point3(origin)
cell_size = dataset.cell_size

# sdf
print('calculating signed distance field ...');
field = signedDistanceField3D(dataset.map, dataset.cell_size)
print('calculating signed distance field done')

# arm: WAM arm
arm = generateArm('SAWYERArm')


# Make PyRobot Object
robot = Robot('sawyer')
robot.arm.go_home()
start_conf = robot.arm.get_joint_angles()
start_conf[0] = np.pi/2

robot.arm.move_to_neutral()
end_conf = robot.arm.get_joint_angles()
end_conf[0] = np.pi/2
start_vel = np.zeros(7)
end_vel = np.zeros(7)


# plot problem setting
figure0 = plt.figure(0)
axis0 = Axes3D(figure0)
axis0.set_title('Problem Settings')
set3DPlotRange(figure0, axis0, dataset)
plotRobotModel(figure0, axis0, arm, start_conf)
plotRobotModel(figure0, axis0, arm, end_conf)
plotMap3D(figure0, axis0, dataset.corner_idx, origin, cell_size)


## settings
total_time_sec = 2.0
total_time_step = 10
total_check_step = 100
delta_t = total_time_sec / total_time_step
check_inter = total_check_step / total_time_step - 1
avg_vel = (end_conf / total_time_step) / delta_t

# GP
Qc = np.identity(7)
Qc_model = noiseModel_Gaussian.Covariance(Qc)

# algo settings
cost_sigma = 0.02
epsilon_dist = 0.2

# noise model
fix_sigma = 0.0001
pose_fix_model = noiseModel_Isotropic.Sigma(7, fix_sigma)
vel_fix_model = noiseModel_Isotropic.Sigma(7, fix_sigma)

# init sdf
sdf = SignedDistanceField(origin_point3, cell_size, field.shape[0], 
                                        field.shape[1], field.shape[2])
for z in range(field.shape[2]):
    sdf.initFieldData(z, field[:,:,z]) #TODO: check this line with its matlab counterpart

#% plot settings
plot_inter_traj = False
plot_inter = 4
if plot_inter_traj:
    total_plot_step = total_time_step * (plot_inter + 1)
else:
    total_plot_step = total_time_step
pause_time = total_time_sec / total_plot_step



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
      graph.push_back(PriorFactorVector(key_pos, start_conf, pose_fix_model))
      graph.push_back(PriorFactorVector(key_vel, start_vel, vel_fix_model))
    elif i==total_time_step:
      graph.push_back(PriorFactorVector(key_pos, end_conf, pose_fix_model))
      graph.push_back(PriorFactorVector(key_vel, end_vel, vel_fix_model))

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
      graph.push_back(PriorFactorVector(key_pos, node_list[i].pose, pose_fix_model))
      graph.push_back(PriorFactorVector(key_vel, start_vel, vel_fix_model))
    elif i==1:
      graph.push_back(PriorFactorVector(key_pos, node_list[i].pose, pose_fix_model))
      graph.push_back(PriorFactorVector(key_vel, end_vel, vel_fix_model))
    
    if i > 0:
      #% cost factor
      graph.push_back(ObstacleSDFFactorArm(
            key_pos, arm, sdf, cost_sigma, epsilon_dist))
      
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
      if check_inter > 0:
        for j in range(1, check_inter+1):
          tau = j * (total_time_sec / total_check_step)
          graph.push_back(ObstacleSDFFactorGPArm(
                    key_pos1, key_vel1, key_pos2, key_vel2,
                    arm, sdf, cost_sigma, epsilon_dist,
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



  # plot final values
  figure2 = plt.figure(2)
  axis2 = Axes3D(figure2)
  axis2.set_title('Result Values')
  plotMap3D(figure2, axis2, dataset.corner_idx, origin, cell_size)
  set3DPlotRange(figure2, axis2, dataset)
  for i in range(total_plot_step):
    conf = path[i]
    plotArm(figure2, axis2, arm.fk_model(), conf, 'b', 2)
    plt.pause(pause_time)


  plt.show()
