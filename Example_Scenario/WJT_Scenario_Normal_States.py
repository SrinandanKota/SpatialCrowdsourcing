# -*- coding: utf-8 -*-

from __future__ import print_function
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import collections
import sys
import datetime

"""
Created on Tue Mar  5 15:31:02 2019

@author: SRINANDAN KOTA
"""

"""Capacitated Vehicle Routing Problem with Time Windows (CVRPTW).
"""

class WJT:
    
    skills={}
    workers={}
    jobs={}
    tasks_by_workers={}
    #workers passing through task locations
    location_and_workers={}
    trajectory={}
    #location of each worker and his task location
    worker_task_locations=collections.OrderedDict()
    worker_task_distances=collections.OrderedDict()
    tasks={}
    #final time after tasks cannot be completed
    cap_time=50
    #limit solutions by number
    search_no_sol=1
    #limit solutions by time
    search_time_limit=250
    
    def __init__(self,workers,jobs,skills):
        self.workers=workers
        self.jobs=jobs
        self.skills=skills
        self.tasks=self.createTasks(jobs)
        self.tasks_by_workers=self.__createTaskList__(workers,self.tasks)
        #creates distance matrix for each worker which can be passed onto vrp 
        self.prepare_depot_model()
        self.__assignTravelPaths__(self.workers)
        self.tasksNotCompleted()

    def createTasks(self,jobs):
         tasks={}
         for jobs_id,jobs_info in jobs.items():
             for task_id,task_info in jobs_info.items():
                 tasks.setdefault(task_id,[])
                 tasks[task_id]=task_info         
         #passed
         #test
         #print(self.tasks)
         return tasks
        
    def __createTaskList__(self,workers,tasks):
         tasks_by_workers={}
         for worker_id, worker_info in workers.items():
             tasks_by_workers.setdefault(worker_id,[])
             for task_id,task_desc in tasks.items():
                 if task_desc[2] in worker_info[1]:
                    tasks_by_workers[worker_id].append(task_id)
         #passed
         #test
         #print(self.tasks_by_workers)
         return tasks_by_workers            
            
    def __assignTravelPaths__(self,workers):
         #Actual trajectories of workers
         start_assignement_vrp = datetime.datetime.now()
         for worker_id in workers.keys():
             self.trajectory.setdefault(worker_id,[])
             curr_worker_traj=self.__vrp__(worker_id)
             if(len(curr_worker_traj)==0):
                print('No task assignment for worker with id = {}'.format(worker_id))
                print("")
             self.trajectory[worker_id].append(curr_worker_traj)
             for task_id in curr_worker_traj:
                 if task_id!=worker_id:
                     self.remove_task_from_workers(task_id)
         end_assignement_vrp = datetime.datetime.now()
         df=end_assignement_vrp-start_assignement_vrp
         print("")
         print("Elapsed time for assignment")
         print(df.total_seconds()*1000)
         print("")
         #passed
         #test    
         #print(self.trajectory)
                    
    def remove_task_from_workers(self,task_id):
        #Remove task once it is assigned to a worker
        for worker_id,task_id_location in self.worker_task_distances.items():
            self.worker_task_distances[worker_id].pop(task_id,-1)             


    def tasksNotCompleted(self):
        #Details of the tasks completed
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
            #calculate manhattan distances between current worker and his tasks
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
       
            
    def create_data_model(self, worker_id):
        
        #calculates for each worker once
        
        data={}
        
        _distances=list(self.worker_task_distances[worker_id].values())
        
        demands=[]
        
        time_windows=[]
        
        #demand for depot is zero
        demands.append(0)
        #time window for worker
        time_windows.append((0,0))
        
        for task_id in self.worker_task_distances[worker_id].keys():
            #demand for task location based on skill, this will determine service time
            if task_id!=worker_id:
                #worker has zero service time at his starting location
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
        horizon = self.cap_time
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
            

    def print_path(self,worker_id, data, routing, assignment):
        """Prints assignment on console"""
        # Inspect solution.
        time_dimension = routing.GetDimensionOrDie('Time')
        total_dist = 0
        time_matrix = 0
        nodes_vis=[]

        print("In the time interval(t1,t2), t1 indicates earliest arrival time and t2 indicates latest departure time of a worker so that worker can meet his task schedules")
        print("")
        
        #maps the node id from software suite to task id in system 
        node_to_task=list(self.worker_task_distances[worker_id].keys())
        
        plan_output = 'Route for worker with id = {0}:\n'.format(worker_id)
        
        for vehicle_id in range(data["num_vehicles"]):
            index = routing.Start(vehicle_id)
            #plan_output = 'Route for vehicle {0}:\n'.format(vehicle_id)
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
                index = assignment.Value(routing.NextVar(index))
            
            node_index = routing.IndexToNode(index)
            #add depot location of worker again
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
        #print('Total Distance of all routes: {0} m'.format(total_dist))
        #print('Total Time of all routes: {0} min'.format(time_matrix))
        
        return nodes_vis


    def __vrp__(self,worker_id):
        
        #keeps track of visited indexes
        vis_nodes=[]
        #tasks assigned in trajectory of this worker
        curr_worker_traj=[]
        
        #maps the node id in software suite to task id in system 
        node_to_task=list(self.worker_task_distances[worker_id].keys())
        
        data = self.create_data_model(worker_id)

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
            #obtain visited indexes
            vis_nodes=self.print_path(worker_id, data, routing, assignment)
            print("")
        
        #form a trajectory
        for node_idx in vis_nodes:
            curr_worker_traj.append(node_to_task[node_idx])
                
        return curr_worker_traj

        
def main():
    print("Normal routing for workers without notifications")
    print("")
    print("")
    skills={'repair':5,'paint':3,'wash':5,'clean':5}
    #initial location,tasks,travelling speed
    workers={1:[(0,0),['repair','wash'],1], \
             2:[(12,2),['paint'],1], \
             3:[(15,2),['wash','clean'],1]}
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
      
    wjt= WJT(workers,jobs,skills)
    
if __name__=='__main__':
    main()
    
        