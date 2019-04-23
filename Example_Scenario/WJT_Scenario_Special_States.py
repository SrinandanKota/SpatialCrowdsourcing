# -*- coding: utf-8 -*-

from __future__ import print_function
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import collections
import sys
import datetime
import time

"""
Created on Tue Mar  5 15:31:02 2019

@author: SRINANDAN KOTA
"""

"""Capacitated Vehicle Routing Problem with Time Windows (CVRPTW).
"""


class WJT_Special:
    
    skills={}
    workers={}
    jobs={}
    tasks_by_workers={}
    #workers passing through task locations
    tasks_and_workers=collections.OrderedDict()
    trajectory=collections.OrderedDict()
    initial_trajectory=collections.OrderedDict()
    #location of each worker and his task location
    worker_task_locations=collections.OrderedDict()
    worker_task_distances=collections.OrderedDict()
    tasks={}
    #workers who are re-routed
    #consider tasks after notifciation time for these workers
    workers_free_not=[]
    #final time after tasks cannot be completed
    cap_time=50
    #limit solutions by time
    search_time_limit=5
    #limit solutions by number
    search_no_sol=1
    #notification time instant
    env_time_at_notf=0
    #parameters to change for simulating fast assignements
    #task not finished id
    task_not_comp_id='J2t1'
    #worker who sends notification
    notification_id=1
    #notification co-ordinates of worker
    notification_x=1
    notification_y=2

    
    def __init__(self,workers,jobs,skills):
        self.workers=workers
        self.jobs=jobs
        self.skills=skills
        self.tasks=self.createTasks(jobs)
        self.tasks_by_workers=self.__createTaskList__(workers,self.tasks)
        #get trajectories without abnormality
        self.__createNormalTrajectories__()
        #creates distance matrix for each worker which can be passed onto vrp 
        self.prepare_depot_model()
        self.tasks_and_workers=self.__matchTasksToLocations__(workers,self.tasks)
        #simulate abnormal situation and simulate re-assignment
        self.simulate_abnormal_situation()
        #print details of tasks completed
        self.tasksNotCompleted()
          
    def createTasks(self,jobs):
         tasks={}
         for jobs_id,jobs_info in jobs.items():
             for task_id,task_info in jobs_info.items():
                 tasks.setdefault(task_id,[])
                 tasks[task_id]=task_info         
         #passed
         #test
         #print(tasks)
         return tasks
         
        
    def __createTaskList__(self,workers,tasks):
         tasks_by_workers=collections.OrderedDict()
         for worker_id, worker_info in workers.items():
             tasks_by_workers.setdefault(worker_id,[])
             for task_id,task_desc in tasks.items():
                 if task_desc[2] in worker_info[1]:
                    tasks_by_workers[worker_id].append(task_id)
         #passed
         #test
         #print(self.tasks_by_workers)
         return tasks_by_workers
    
    #gets service time at a task location
    def getServiceTime(self,task_id):
        if task_id not in list(self.workers.keys()):
            return int(self.skills[self.tasks[task_id][2]])
        else:
            return 0

    def get_normal_trajectory(self,worker_id, data, routing, assignment):
        """Prints assignment on console"""
        # Inspect solution.
        time_dimension = routing.GetDimensionOrDie('Time')
        total_dist = 0
        time_matrix = 0
        #visted nodes
        nodes_vis=[]

        #update task locations and arrival and departure times in initial trajectory structure
        self.initial_trajectory.setdefault(worker_id,collections.OrderedDict())
        
        #maps the node id from software suite to task id in system 
        node_to_task=list(self.worker_task_distances[worker_id].keys())
        
        plan_output = 'Route for worker with id = {0}:\n'.format(worker_id)
        
        for vehicle_id in range(data["num_vehicles"]):
            index = routing.Start(vehicle_id)
            route_dist = 0
            while not routing.IsEnd(index):
                #keep adding visited indexes
                nodes_vis.append(index)
                node_index = routing.IndexToNode(index)
                next_node_index = routing.IndexToNode(
                        assignment.Value(routing.NextVar(index)))
                route_dist += routing.GetArcCostForVehicle(node_index, next_node_index, vehicle_id)
                time_var = time_dimension.CumulVar(index)
                time_min = assignment.Min(time_var)
                time_max = assignment.Max(time_var)
                #plan_output += ' {0} Time({1},{2}) ->'.format(node_index, time_min, time_max)
                plan_output += ' {0} Time({1},{2}) ->'.format(node_to_task[node_index], time_min, time_max)
                #print(node_to_task[index])
                self.initial_trajectory[worker_id].setdefault(node_to_task[index],[])
                self.initial_trajectory[worker_id][node_to_task[index]]=[time_min, \
                                       time_max]
                index = assignment.Value(routing.NextVar(index))

            
            node_index = routing.IndexToNode(index)
            #add starting location of worker again
            nodes_vis.append(node_index)
            time_var = time_dimension.CumulVar(index)
            route_time = assignment.Value(time_var)
            time_min = assignment.Min(time_var)
            time_max = assignment.Max(time_var)
            total_dist += route_dist
            time_matrix += route_time
            #plan_output += ' {0} Time({1},{2}) \n'.format(node_to_task[node_index], time_min, time_max)
            #plan_output += 'Distance of the route: {0} m\n'.format(route_dist)
            #plan_output += 'Time of the route: {0} min\n'.format(route_time)
            #print(plan_output)
            #print(node_to_task[node_index])
            self.initial_trajectory.setdefault(node_to_task[node_index],[])
            self.initial_trajectory[worker_id][node_to_task[node_index]]=[time_min, \
                                   time_max]

        #print('Total Distance of all routes: {0} m'.format(total_dist))
        #print('Total Time of all routes: {0} min'.format(time_matrix))
        
        return nodes_vis     
    

    def __vrpn__(self,worker_id):
        
        #keeps track of visited indexes which are modelled tasks
        vis_nodes=[]
        #tasks assigned to trajectory of this worker
        curr_worker_traj=[]
        
        #maps the node id from software suite to task id in system 
        node_to_task=list(self.worker_task_distances[worker_id].keys())
        
        #prepare data model for sofwtare suite
        data = self.create_data_model(0,worker_id)

        # Create Routing Model
        routing = pywrapcp.RoutingModel(data["num_locations"], data["num_vehicles"], data["depot"])
        
        # Add Time Window constraint
        time_callback = self.create_time_callback(data)
        
        #Time
        #Define weight of each edge
        routing.SetArcCostEvaluatorOfAllVehicles(time_callback)
        
        
        self.add_time_window_constraints(routing, data, time_callback)
        
        # Setting first solution heuristic (cheapest addition).
        search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
        #search_parameters.time_limit_ms = self.search_time_limit
        search_parameters.solution_limit=self.search_no_sol
        search_parameters.local_search_metaheuristic = (                    \
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
        
        # Solve the problem.
        assignment = routing.SolveWithParameters(search_parameters)
        if assignment:
            #get visted nodes of worker
            vis_nodes=self.get_normal_trajectory(worker_id, data, routing, assignment)
            
        #prepare trajectory of the worker
        for node_idx in vis_nodes:
            curr_worker_traj.append(node_to_task[node_idx])
            
            
        return curr_worker_traj


     
    def __assignTravelPathsNormal__(self,workers):
         #actual trajectories of workers
         for worker_id in workers.keys():
             curr_worker_traj=self.__vrpn__(worker_id)
             if(len(curr_worker_traj)==0):
                print('No task assignment for worker with id = {}'.format(worker_id))
                print("")
             for task_id in curr_worker_traj:
                 if task_id!=worker_id:
                     #remove assigend tasks
                     self.remove_task_from_workers(task_id)
         #passed
         #test    
         #print(self.trajectory)        
        
        
     
    def __createNormalTrajectories__(self):
        self.prepare_depot_model()
        self.__assignTravelPathsNormal__(self.workers)
        #self.initial_trajectory=collections.OrderedDict(self.trajectory)
        #self.trajectory=self.initial_trajectory
        
        #test passed
        #print("Normal Trajectories")
        #print(self.initial_trajectory)
          

             
    def __matchTasksToLocations__(self,workers,tasks):
        #get workers who are can visit a task
        tasks_and_workers=collections.OrderedDict()
        for task_id, task_info in tasks.items():
            worker_ids=[]
            tasks_and_workers.setdefault(task_id,[])
            worker_ids=tasks_and_workers[task_id]
            for worker_id, worker_info in workers.items():
                if task_info[2] in worker_info[1]:
                    if worker_id not in worker_ids:
                        worker_ids.append(worker_id)
            tasks_and_workers[task_id]=worker_ids
        
        #passes
        #test
        #print(tasks_and_workers)
        
        return tasks_and_workers
                    
    def remove_task_from_workers(self,task_id):
        #remove assigned task
        for worker_id,task_id_location in self.worker_task_distances.items():
            self.worker_task_distances[worker_id].pop(task_id,-1)

    def tasksNotCompleted(self):
        #details of completed tasks
        print("")
        tasks_not_completed=[]
        for worker_id,task_info in self.worker_task_distances.items():
            for task_id in task_info:
                if task_id!=worker_id:
                    tasks_not_completed.append(task_id)
        if(len(set(tasks_not_completed))==0):
            print("All tasks completed")
            print("")
        else:
            print("Tasks not completed by workers are listed below")
            print(set(tasks_not_completed))
            print("")
            print("")
            print("Percentage of tasks not completed")
            print((len(set(tasks_not_completed))/len(self.tasks.keys()))*100)
        
    def create_distances(self,vrp_id,worker_id):
        curr_distances=[]
        curr_worker_task_loc=self.worker_task_locations[worker_id]
        for other_id in curr_worker_task_loc.keys():
            #calculate manhattan distances for current worker and his tasks
            curr_distances.append(abs(curr_worker_task_loc[vrp_id][0]-curr_worker_task_loc[other_id][0]) \
            +abs(curr_worker_task_loc[vrp_id][1]-curr_worker_task_loc[other_id][1]))
        return curr_distances
             
    def prepare_depot_model(self):
        #prepare structure to hold location co-ordinates
        for worker_id,worker_tasks in self.tasks_by_workers.items():
            curr_task_loc=collections.OrderedDict()
            #add worker location, this is depot
            self.worker_task_locations.setdefault(worker_id,{})
            curr_task_loc.setdefault(worker_id,self.workers[worker_id][0])
            for task_id in worker_tasks:
                curr_task_loc.setdefault(task_id,())
                curr_task_loc[task_id]=self.tasks[task_id][0]   
            self.worker_task_locations[worker_id]=curr_task_loc
        
        #prepare structure to hold disatnces between locations
        for worker_id,tasks_worker_locations in self.worker_task_locations.items():
            self.worker_task_distances.setdefault(worker_id,{})
            curr_task_distances=collections.OrderedDict()
            for vrp_id in tasks_worker_locations.keys():
                curr_task_distances.setdefault(vrp_id,[])
                curr_task_distances[vrp_id]=self.create_distances(vrp_id,worker_id)
            self.worker_task_distances[worker_id]=curr_task_distances
            
        #passed
        #test
        #print(self.worker_task_distances[1])
       
            
    def create_data_model(self, earliest_dep_time, worker_id):
        
        #calculates for each worker once
        
        data={}                    
        
        _distances=list(self.worker_task_distances[worker_id].values())
        
        demands=[]
        
        time_windows=[]
        
        #demand for depot is zero
        demands.append(0)
         
        #new time windows for re-routed workers from notification time
        if (worker_id in self.workers_free_not):
            time_windows.append((self.env_time_at_notf,self.env_time_at_notf))
        else:
            time_windows.append((0,0))

        
        for task_id in self.worker_task_distances[worker_id].keys():
            #demand for task location based on skill, this will determine service time
            if task_id!=worker_id:
                demands.append(self.skills[self.tasks[task_id][2]])
                time_windows.append(self.tasks[task_id][1])                                 
                
                
        data["distances"]=_distances
        
        data["num_locations"]=len(_distances)
        
        data["num_vehicles"]= 1
        
        data["depot"] = 0

        data["demands"] = demands

        data["time_windows"] = time_windows

        data["time_per_demand_unit"] = 1

        data["vehicle_speed"] = self.workers[worker_id][2]

        return data

    def create_distance_callback(self,data):
        """Creates callback to return distance between points."""
        distances = data["distances"]                
          
        def distance_callback(from_node, to_node):
            """Returns the manhattan distance between the two nodes"""
            return distances[from_node][to_node]
        
        return distance_callback              
                 
    def create_time_callback(self,data):
        """Creates callback to get total times between locations."""
        def service_time(node):
            """Gets the service time for the specified location."""
            return data["demands"][node] * data["time_per_demand_unit"]
           
        def travel_time(from_node, to_node):
            """Gets the travel times between two locations."""
            travel_time = data["distances"][from_node][to_node] / data["vehicle_speed"]
            return travel_time
      
        def time_callback(from_node, to_node):
            """Returns the total time between the two nodes"""
            serv_time = service_time(from_node)
            trav_time = travel_time(from_node, to_node)
            return serv_time + trav_time
      
        return time_callback         
         
    def add_time_window_constraints(self,routing, data, time_callback):
        """Add time window constraints."""
        time = "Time"
        # Need to force it to zero
        horizon= self.cap_time
        routing.AddDimension(
                time_callback,
                horizon, # allow waiting time
                horizon, # maximum time per vehicle
                False, # Don't force start cumul to zero. This doesn't have any effect in this example,
                # since the depot has a start window of (0, 0).
                time)

        time_dimension = routing.GetDimensionOrDie(time)
        for location_node, location_time_window in enumerate(data["time_windows"]):
            index = routing.NodeToIndex(location_node)
            time_dimension.CumulVar(index).SetRange(location_time_window[0], location_time_window[1])


    def get_time_travelled_bw_points(self,not_worker_id,x1,x2,y1,y2):
        #travel time of worker
        return int((abs(x1-x2)+abs(y1-y2))/self.workers[not_worker_id][2])

    
    
    def get_env_time(self,not_worker_id,task_comp_id,task_not_comp_id, notification_x,notification_y, previous_compl_task_max_time):
        
        #id of completed task from initial trajectory
        task_comp_id_x=self.worker_task_locations[not_worker_id][task_comp_id][0]
        
        #id of uncompleted task from initial trajectory
        task_comp_id_y=self.worker_task_locations[not_worker_id][task_comp_id][1]
        
        #travel time
        curr_travel_time=self.get_time_travelled_bw_points(not_worker_id,task_comp_id_x,notification_x,task_comp_id_y,notification_y)
        
        #get service time of previous visited location
        if task_comp_id not in list(self.workers.keys()):
            service_time_prev_comp_task=self.skills[self.tasks[task_comp_id][2]]
        else:
            service_time_prev_comp_task=0  
            
        curr_env_time=previous_compl_task_max_time+curr_travel_time
        
        #test passed
        #print(task_comp_id)
        #print("x")
        #print(task_comp_id_x)
        #print("y")
        #print(task_comp_id_y)
        #print("notification location")
        #print("x")
        #print(notification_x)
        #print("y")
        #print(notification_y)        
        #print("travel time")
        #print(curr_travel_time)
        #print("service time of previous location")
        #print(service_time_prev_comp_task)
        #print("arrival time of previous location")
        #print(previous_compl_task_min_time)
        #print("total time spent in last location")
        #print(previous_compl_task_min_time+service_time_prev_comp_task)
        #print("current environment time")
        #print(curr_env_time)
        
        return curr_travel_time,curr_env_time
    
    def print_path_workers_not_reassigned(self,reassigned_worker_ids):
        
        #prints trajectories of workers not re-routed
        #renoves their ompleted tasks     
        for init_worker_id,init_worker_trajectory in self.initial_trajectory.items():
            plan_output=''
            if init_worker_id not in reassigned_worker_ids:
               plan_output+='{0} ({1},{2})->'.format(init_worker_id,0,0)
               for loc_id, loc_time_interval in init_worker_trajectory.items():
                   if loc_id not in list(self.workers.keys()):
                       plan_output+='{0} ({1},{2})->'.format(loc_id,loc_time_interval[0],loc_time_interval[1])
                       #remove task occurence from all workers
                       self.remove_task_from_workers(loc_id)
               last_loc_time_interval=list(init_worker_trajectory.values())[0]
               plan_output+='{0} ({1},{2})\n'.format(init_worker_id,last_loc_time_interval[0],last_loc_time_interval[1])
            print(plan_output)

     
    def print_path_worker_notifes(self, curr_env_time, not_worker_id):
        
        #prints initial trajectories for worker who sends notifictaion
        #renoves their ompleted tasks 
        plan_output=''
        plan_output+='{0} ({1},{2})->'.format(not_worker_id,0,0)
        for loc_id,loc_time_interval in self.initial_trajectory[not_worker_id].items():
            time_max=loc_time_interval[1]
            if curr_env_time>time_max:
               if loc_id not in list(self.workers.keys()):
                  plan_output+='{0} ({1},{2})->'.format(loc_id,loc_time_interval[0],loc_time_interval[1])
                  #remove task occurence from all workers
                  self.remove_task_from_workers(loc_id)
        print(plan_output)

    def print_path_worker_busy(self,worker_id):
        
        #prints details of tasks copleted by busy workers
        #renoves their ompleted tasks 
        plan_output=''
        plan_output+='{0} ({1},{2})->'.format(worker_id,0,0)
        for loc_id,loc_time_interval in self.initial_trajectory[worker_id].items():
            if loc_id not in list(self.workers.keys()):
               plan_output+='{0} ({1},{2})->'.format(loc_id,loc_time_interval[0],loc_time_interval[1])
               #remove task occurence from all workers
               self.remove_task_from_workers(loc_id)  
        last_loc_time_interval=list(self.initial_trajectory[worker_id].values())[0]
        plan_output+='{0} ({1},{2})\n'.format(worker_id,last_loc_time_interval[0],last_loc_time_interval[1])
        print(plan_output)               
               
        
    def workers_moving_and_status(self,curr_env_time,capable_worker_ids):
        
        #details of free workers
        free_workers=collections.OrderedDict()
        curr_worker_traj=collections.OrderedDict()
        for worker_id in capable_worker_ids:
            curr_worker_traj.setdefault(worker_id,[])
            #trajectories start from initial location and reach back
            curr_worker_traj[worker_id]=list(self.initial_trajectory[worker_id].keys())
        
        #update new locations for these workers
        for worker_id,worker_traj in curr_worker_traj.items():
            print('Updates of Current worker {0} till time t = {1}'.format(worker_id, curr_env_time))
            for i in range(len(worker_traj)):
                task_id=worker_traj[i]
                if task_id not in list(self.workers.keys()):
                    time_min=int(self.initial_trajectory[worker_id][task_id][0])
                    time_max=int(self.initial_trajectory[worker_id][task_id][1])
                elif task_id in list(self.workers.keys()):
                    time_min=0
                    time_max=0
                if time_min==time_max:
                    print('Left from location of {0}'.format(task_id))
                    if(len(self.initial_trajectory[worker_id].keys())==1):
                        print('Worker {0} will be available'.format(worker_id))
                if curr_env_time>=time_min and curr_env_time<time_max:
                    print('Currently at location of {0}'.format(task_id))
                    print('Maybe Busy')
                    print('Retains his initial trajectory and not considerd for re-assignement')
                    print("His initial trajectory")
                    self.print_path_worker_busy(worker_id)
                    print("")
                    break
                elif curr_env_time>time_max:
                    if task_id not in list(self.workers.keys()):
                        print('worker finished task {0}'.format(task_id))
                        self.remove_task_from_workers(task_id)
                elif curr_env_time<time_min:
                    print('Worker {0} will be available'.format(worker_id))
                    free_workers.setdefault(worker_id,{})
                    free_workers[worker_id]=[worker_traj[i-1],worker_traj[i]]
                    break
            print("")
         
        #test passed    
        #print("List of free workers who can do current unfinished task")
        #print(free_workers)
        
        #add workers who are currently moving and who notified
        self.workers_free_not=list(free_workers.keys())
        self.workers_free_not.append(self.notification_id)  
        
        return free_workers

    def get_curr_pos(self, worker_id, curr_travel_time, capable_free_workers_tasks):
        
        #new positions of workers at notification time
        prev_loc_id=capable_free_workers_tasks[0]
        next_loc_id=capable_free_workers_tasks[1]
        prev_loc_x=self.worker_task_locations[worker_id][prev_loc_id][0]
        prev_loc_y=self.worker_task_locations[worker_id][prev_loc_id][1]
        next_loc_x=self.worker_task_locations[worker_id][next_loc_id][0]
        next_loc_y=self.worker_task_locations[worker_id][next_loc_id][1]
        curr_loc_x=0
        curr_loc_y=0
        rem_travel_time=0

        if next_loc_x>prev_loc_x:
            if prev_loc_x+curr_travel_time<next_loc_x:
                curr_loc_x=prev_loc_x+curr_travel_time
            elif prev_loc_x+curr_travel_time>=next_loc_x:
                curr_loc_x=next_loc_x
                rem_travel_time=curr_travel_time-(next_loc_x-prev_loc_x)
            if next_loc_y>=prev_loc_y:
                curr_loc_y=prev_loc_y+rem_travel_time
            elif prev_loc_y>next_loc_y:
                curr_loc_y=prev_loc_y-rem_travel_time
        
        if next_loc_x<prev_loc_x:
           if next_loc_x+curr_travel_time<prev_loc_x:
              curr_loc_x=next_loc_x+curr_travel_time
           elif next_loc_x+curr_travel_time>=prev_loc_x:
              curr_loc_x=next_loc_x
              rem_travel_time=curr_travel_time-(prev_loc_x-next_loc_x)
           if next_loc_y>=prev_loc_y:
              curr_loc_y=prev_loc_y+rem_travel_time
           if prev_loc_y>next_loc_y:
              curr_loc_y=prev_loc_y-rem_travel_time
              
        return(curr_loc_x,curr_loc_y)
        

    def create_distances_of_capable_workers(self,vrp_id, worker_id, capable_worker_task_locations):       
        
        #get new distances from new locations of re-routed workers
        curr_distances=[]
        curr_worker_task_loc=capable_worker_task_locations[worker_id]
        for other_id in curr_worker_task_loc.keys():
            #calculate manhattan distances for current worker and his tasks
            curr_distances.append(abs(curr_worker_task_loc[vrp_id][0]-curr_worker_task_loc[other_id][0]) \
            +abs(curr_worker_task_loc[vrp_id][1]-curr_worker_task_loc[other_id][1]))
        return curr_distances 

    
    def update_locations_recompute_distance(self,curr_travel_time,capable_free_workers):
        
        #small copy of worker_task_locations
        capable_worker_task_locations=collections.OrderedDict()
        #small copy of worker_task_distances
        capable_worker_task_distances=collections.OrderedDict()
        for worker_id,tasks_worker_locations in self.worker_task_locations.items():
            if worker_id in capable_free_workers.keys():
               capable_worker_task_locations.setdefault(worker_id,{}) 
               capable_worker_task_locations[worker_id]=collections.OrderedDict(tasks_worker_locations)
        
        for worker_id,worker_task_loc in capable_worker_task_locations.items():
            for vrp_id in worker_task_loc.keys():
                if vrp_id==worker_id and vrp_id!=self.notification_id:
                   #updating poistions of free workers
                   capable_worker_task_locations[worker_id][vrp_id]= \
                   self.get_curr_pos(worker_id,curr_travel_time,capable_free_workers[vrp_id])
                   print('Worker {} is currently not busy'.format(worker_id))
                   print('He is at {}'.format(capable_worker_task_locations[worker_id][vrp_id]))
                   print('Updated his current location'.format(worker_id))
                   print("His new printed trajectories will start from current environment time", end='')
                   print(" as he is rerouted to check if he can perform the unfinished task")
                   print("")
                elif vrp_id==self.notification_id:
                   #updating position of notifier
                   capable_worker_task_locations[worker_id][vrp_id]=(self.notification_x,self.notification_y) 
                   print('Worker {0} cannot do task {1}'.format(worker_id,self.task_not_comp_id))
                   print('He is at {}'.format(capable_worker_task_locations[worker_id][vrp_id]))
                   print('Updated his current location'.format(worker_id))
                   print("His new printed trajectories will start from current environment time", end='')
                   print(" as he is rerouted to perform other tasks")
                   print("")
         
        #creating distance arrays for workers who are reassigned
        for worker_id,tasks_worker_locations in capable_worker_task_locations.items():
            capable_worker_task_distances.setdefault(worker_id,{})
            curr_task_distances=collections.OrderedDict()
            for vrp_id in tasks_worker_locations.keys():
                curr_task_distances.setdefault(vrp_id,[])
                curr_task_distances[vrp_id]=self.create_distances_of_capable_workers(vrp_id,worker_id,capable_worker_task_locations)
            capable_worker_task_distances[worker_id]=curr_task_distances

        
        #removing all completed tasks from above copy 
        for worker_id in capable_worker_task_distances.keys():
                for worker_task_id in list(capable_worker_task_distances[worker_id].keys()):
                   if worker_task_id not in list(self.worker_task_distances[worker_id].keys()):
                       if worker_task_id not in list(self.workers.keys()):
                           capable_worker_task_distances[worker_id].pop(worker_task_id,-1)

        
        #updating worker_task_distance
        for worker_id,tasks_worker_dist in capable_worker_task_distances.items():
            self.worker_task_distances[worker_id]=collections.OrderedDict(capable_worker_task_distances[worker_id])
        
        return capable_free_workers.keys()    

    def update_locations_cap_workers_not_initially_assigned(self,curr_travel_time,capable_worker_ids):
        
        
        #small copy of worker_task_locations
        capable_worker_task_locations=collections.OrderedDict()
        #small copy of worker_task_distances
        capable_worker_task_distances=collections.OrderedDict()
        #workers with no initial assignements and who can do unfinished tasks
        workers_no_assignments=[]
        
        #get capable workers without initial assignments
        for worker_id in capable_worker_ids:
            if len(self.initial_trajectory[worker_id].keys())==1:
                workers_no_assignments.append(worker_id)  
                
        #details of free workers
        print("Workers who can perform this task and were initially idle")
        if len(workers_no_assignments)!=0:
            print(workers_no_assignments)
            print("")
        else:
            print("No such worker is idle")
            print("")
            return []
        print("Updating their locations and getting their statuses")
        print("")
        
        for worker_id,tasks_worker_locations in self.worker_task_locations.items():
            if worker_id in workers_no_assignments:
               capable_worker_task_locations.setdefault(worker_id,{}) 
               capable_worker_task_locations[worker_id]=collections.OrderedDict(tasks_worker_locations)
 

        #update new locations at notification time
        for worker_id,worker_task_loc in capable_worker_task_locations.items():
            for vrp_id in worker_task_loc.keys():
                if vrp_id==worker_id and vrp_id!=self.notification_id:
                   #Move current worker by some disance in x and y co-ordinate from initial
                   capable_worker_task_locations[worker_id][vrp_id]= \
                   (self.workers[worker_id][0][0]+curr_travel_time-int(curr_travel_time/2), \
                    self.workers[worker_id][0][1]+int(curr_travel_time/2))
                   print('Worker {} is currently not busy'.format(worker_id))
                   print('He is at {}'.format(capable_worker_task_locations[worker_id][vrp_id]))
                   print('Updated his current location'.format(worker_id))
                   print("His new printed trajectories will start from current environment time", end='')
                   print(" as he is re-routed to check if he can perform the unfinished task")
                   print("")      

        #creating distance arrays for workers who are reassigned
        for worker_id,tasks_worker_locations in capable_worker_task_locations.items():
            capable_worker_task_distances.setdefault(worker_id,{})
            curr_task_distances=collections.OrderedDict()
            for vrp_id in tasks_worker_locations.keys():
                curr_task_distances.setdefault(vrp_id,[])
                curr_task_distances[vrp_id]=self.create_distances_of_capable_workers(vrp_id,worker_id,capable_worker_task_locations)
            capable_worker_task_distances[worker_id]=curr_task_distances

        #removing all completed tasks from above copy 
        for worker_id in capable_worker_task_distances.keys():
                for worker_task_id in list(capable_worker_task_distances[worker_id].keys()):
                   if worker_task_id not in list(self.worker_task_distances[worker_id].keys()):
                       if worker_task_id not in list(self.workers.keys()):
                           capable_worker_task_distances[worker_id].pop(worker_task_id,-1)
        

        #updating worker_task_distance
        for worker_id,tasks_worker_dist in capable_worker_task_distances.items():
            self.worker_task_distances[worker_id]=collections.OrderedDict(capable_worker_task_distances[worker_id])


        #assign workers who are capable and with no initial trajectories
        #start from notification time
        for worker_id in workers_no_assignments:
            self.workers_free_not.append(worker_id) 
            
            
        return  workers_no_assignments

    
    def handleNotifications(self,curr_travel_time, curr_env_time, not_worker_id,task_comp_id,task_not_comp_id, notification_x,notification_y, previous_comp_task_max_time):  
        
        #simulate abnormal assignments
        print('Worker {0} issued a  notification about {1} at ({2},{3}) and at time t = {4} '.format(not_worker_id, task_not_comp_id,notification_x,notification_y,curr_env_time))
        print("")
        print('Intial trajectory of worker {0} till time t = {1}'.format(not_worker_id,curr_env_time))
        #prints path finished till current envrionment time by worker who notifies 
        self.print_path_worker_notifes(curr_env_time, not_worker_id)
        print("")
        print('Displaying updates for workers who can perform this task at current time')
        print("")
        
        #get workers who can do task other than current worker
        capable_worker_ids=[]
        for worker_can_comp in self.tasks_and_workers[task_not_comp_id]:
            capable_worker_ids.append(worker_can_comp)
        capable_worker_ids.remove(not_worker_id)
        
        #capable free workers have workers who are capable, not busy
        capable_free_workers=self.workers_moving_and_status(curr_env_time,capable_worker_ids)
        #update locations of all above capable workers 
        print('Updating locations for these workers and getting their statuses at time t = {}'.format(curr_env_time))
        print("")
        self.update_locations_recompute_distance(curr_travel_time,capable_free_workers)
        
        #updating location of notifier
        capable_not_worker=collections.OrderedDict()
        capable_not_worker.setdefault(not_worker_id,[task_comp_id,task_not_comp_id])
        self.update_locations_recompute_distance(curr_travel_time, \
             capable_not_worker)
        
        #updating locations of capable workers who were not initially assigned trajectories
        workers_no_assignments=self.update_locations_cap_workers_not_initially_assigned(curr_travel_time,capable_worker_ids)
        
        #removing unreachable task from worker who notifies
        #to avoid re-assigning of this task
        self.worker_task_distances[not_worker_id].pop(task_not_comp_id,-1)
        
        #get workers who are re-assigned
        workers_reassigned=[]
        workers_reassigned.append(not_worker_id)
        for worker_id in capable_free_workers.keys():
            workers_reassigned.append(worker_id)
        for worker_id in workers_no_assignments:
            workers_reassigned.append(worker_id)
        
        return workers_reassigned, curr_env_time


    def print_path_for_reassigned(self,worker_id, data, routing, assignment):
        """Prints assignment on console"""
        # Inspect solution.
        time_dimension = routing.GetDimensionOrDie('Time')
        total_dist = 0
        time_matrix = 0
        #visietd nodes
        nodes_vis=[]
        
        #maps node id from software suite to task id in system 
        node_to_task=list(self.worker_task_distances[worker_id].keys())
        
        plan_output = 'Route for worker with id = {0}:\n'.format(worker_id)
        
        for vehicle_id in range(data["num_vehicles"]):
            index = routing.Start(vehicle_id)
            #plan_output = 'Route for vehicle {0}:\n'.format(vehicle_id)
            route_dist = 0
            while not routing.IsEnd(index):
                #keep adding visited indices
                nodes_vis.append(index)
                node_index = routing.IndexToNode(index)
                next_node_index = routing.IndexToNode(
                        assignment.Value(routing.NextVar(index)))
                route_dist += routing.GetArcCostForVehicle(node_index, next_node_index, vehicle_id)
                time_var = time_dimension.CumulVar(index)
                time_min = assignment.Min(time_var)
                time_max = assignment.Max(time_var)
                previous_comp_task_max_time=time_max
                #print(time_max)
                #plan_output += ' {0} Time({1},{2}) ->'.format(node_index, time_min, time_max)
                plan_output += ' {0} Time({1},{2}) ->'.format(node_to_task[node_index], time_min, time_max)
                index = assignment.Value(routing.NextVar(index))
            
            node_index = routing.IndexToNode(index)
            #initial location is added again, duplicate element in node list
            nodes_vis.append(node_index)
            time_var = time_dimension.CumulVar(index)
            route_time = assignment.Value(time_var)
            time_min = assignment.Min(time_var)
            time_max = assignment.Max(time_var)
            total_dist += route_dist
            time_matrix += route_time
            plan_output += ' {0} Time({1},{2}) \n'.format(node_to_task[node_index], time_min, time_max)
            #plan_output += 'Distance of the route: {0} m\n'.format(route_dist)
            #plan_output += 'Time of the route: {0} min\n'.format(route_time)
            print(plan_output)
            
        return nodes_vis
    
    
    
    def __vrp_new_reassigned__(self,worker_id):
        
        #keeps track of visited indexes
        vis_nodes=[]
        #tasks assigned to trajectory of this worker
        curr_worker_traj=[]
        
        #maps node_idx in vrp to task identification in system 
        node_to_task=list(self.worker_task_distances[worker_id].keys())
        
        data = self.create_data_model(0, worker_id)

        # Create Routing Model
        routing = pywrapcp.RoutingModel(data["num_locations"], data["num_vehicles"], data["depot"])
        
        # Add Time Window constraint
        time_callback = self.create_time_callback(data)
        
        #Time
        #Define weight of each edge
        routing.SetArcCostEvaluatorOfAllVehicles(time_callback)
        
        
        self.add_time_window_constraints(routing, data, time_callback)
        
        
        # Setting first solution heuristic (cheapest addition).
        search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
        #search_parameters.time_limit_ms = self.search_time_limit
        search_parameters.solution_limit=self.search_no_sol
        search_parameters.local_search_metaheuristic = (                    \
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
        
        # Solve the problem.
        assignment = routing.SolveWithParameters(search_parameters)
        if assignment:
            #get visited node indexes
            vis_nodes=self.print_path_for_reassigned(worker_id, data, routing, assignment)
            print("")
        
        #form current trajectory
        for node_idx in vis_nodes:
            curr_worker_traj.append(node_to_task[node_idx])
            
        return curr_worker_traj
    
    
    
    def print_path_workers_reassigned(self,reassigned_worker_ids):
        #print paths for reassigned workers
        #remove assigned tasks
        curr_worker_traj=[]
        for worker_id in reassigned_worker_ids:
            curr_worker_traj=self.__vrp_new_reassigned__(worker_id)
            for task_id in curr_worker_traj:
                if task_id!=worker_id:
                    self.remove_task_from_workers(task_id)
        

    
    def simulate_abnormal_situation(self):
        
        #re-assignemnt task starts
        start_assignment_vrp = datetime.datetime.now()
        
        #worker who notifies
        not_worker_id=self.notification_id
        
        #get task previous task which worker has completed before notification
        initial_traj_not_id=list(self.initial_trajectory[self.notification_id].keys())
        
        #task not completed by notifier
        task_not_comp_id=self.task_not_comp_id
        
        #previous location latest departure time
        for i in range(len(initial_traj_not_id)):
            if initial_traj_not_id[i]==self.task_not_comp_id:
                task_comp_id=initial_traj_not_id[i-1]
                if task_comp_id in (self.workers.keys()):
                   previous_comp_task_max_time=0
                else:
                    previous_comp_task_max_time=self.initial_trajectory[self.notification_id][task_comp_id][1]
                
        notification_x=self.notification_x
        notification_y=self.notification_y

        
        #current travel time from previous location to notification location and current time in the enviornment
        curr_travel_time, curr_env_time=self.get_env_time(not_worker_id,task_comp_id,task_not_comp_id, \
        notification_x,notification_y, previous_comp_task_max_time)
        
        #get workers who can do task including worker who notifies
        reassigned_worker_ids=self.tasks_and_workers[task_not_comp_id]
        
        #initail trajectories of workers not re-assigned
        print("Printing initial trajectories for workers")
        print("")
        self.print_path_workers_not_reassigned(reassigned_worker_ids)

        print('Notifictaion received about task {0}  by worker {1}\n'.format(task_not_comp_id, not_worker_id))
        
        print('Workers considered for rerouting because of this task {}'.format(task_not_comp_id))
        if len(reassigned_worker_ids)!=0:
            print(reassigned_worker_ids)
        else:
            print("None")
        print("")
        
        print("Printing updates for workers who have been affected by this notification")
        print("")
        #get workers who are re-assigned and notifcation time instant
        workers_reassigned,not_time=self.handleNotifications(curr_travel_time, curr_env_time, \
                      self.notification_id,task_comp_id,self.task_not_comp_id, \
                      self.notification_x,self.notification_y, previous_comp_task_max_time)
        
        #Updating notification time so that re-assigned workers start from this time
        self.env_time_at_notf=not_time
        print("Printing rest of trajectories for workers who were considered for re-assignment")
        print("")
        print('New trajectories start from time t = {} '.format(not_time))
        print("They include tasks completed after time t= {}".format(not_time))
        print("")
        print("Displaying rest of the trajectories")
        print("")
        
        #print path for re-assigned workers
        self.print_path_workers_reassigned(workers_reassigned)

        #re-assignemnt ends
        end_assignment_vrp = datetime.datetime.now()
        
        df=end_assignment_vrp-start_assignment_vrp
        print("")
        print("Elapsed time for re-assignment of tasks and workers")
        print(df.total_seconds()*1000)
        print("")

        
def main():
    print("Routing for workers but when a worker cannot reach a task")
    print("")
    print("Scenario with state change that is not previously determined")
    print("")
    print("")
    skills={'repair':5,'paint':3,'wash':5,'clean':5}
    #initial location,tasks,travelling speed
    workers={1:[(0,0),['repair','wash'],1], \
             2:[(12,2),['paint'],1], \
             3:[(15,2),['wash','clean'],1]}
    #job id,task id, location, times, skill
    jobs={'J1':{'J1t1':[(0,2),(0,10),'repair'],'J1t2':[(0,2),(10,15),'paint']},  \
          'J2':{'J2t1':[(2,2),(0,20),'wash'],'J2t2':[(2,2),(20,30),'clean']}    \
         }
    print("Skills in this environment->(Skill:Time)")
    print(skills)
    print("")
    print("")
    print("Worker information in this environment->(Initial Location, Skills, Speed)")
    print(workers)
    print("")
    print("")
    print("Job information in this environment->(Job_id:[Task_id:[Location, Time interval, skill]])")
    print(jobs)
    print("")
    print("")
    
    wjt= WJT_Special(workers,jobs,skills)
    
if __name__=='__main__':
    main()
        