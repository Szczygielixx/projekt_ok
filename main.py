import random
import time
from enum import Enum
import heapq

time0 = time.time()
ID = 0 
trucks = []
cities = []
number_of_packages = 0
c = 50 #number of cities
max_distance = 15 # maximal distance between cities
trucks_number = 25 #number of trucks
capacity = 25 #capacity of a single truck 
max_weight = 9 #maximal weight of a single package


class States(Enum):
    getting = 1
    riding = 2
    delivering = 3


class Truck:
    def __init__(self, truck_ID, curr_position, capacity, map):
        self.ID = truck_ID
        self.curr_position = curr_position
        self.capacity = capacity
        self.map = map
        self.orders = []
        self.weight = 0
        self.delta_t = 0
        self.state = States.getting
        self.total_distance = 0
        self.courses = 0
        self.ChooseAlgorithm = Truck.choose_orders_inorder
        self.ChooseDrivingAlgorithm = Truck.choose_city_dijkstra

    def update(self, timestep):
        self.delta_t -= timestep
        if(self.delta_t > 0):
            return
        if self.state == States.getting:
            #self.printAction()
            self.delta_t = 1
            self.pass_orders()
            self.wez_paczuche(self.ChooseAlgorithm(self))
            self.state = States.riding
        elif self.state == States.riding:
            #self.printAction()
            global cities
            destination = self.ChooseDrivingAlgorithm(self)
            self.drive(destination)
            self.state = States.delivering
        else:
            #self.printAction()
            self.delta_t = 1
            self.deliver(self.curr_position)
            self.state = States.getting

    def printAction(self):
        print("Ciężarówka: ", self.ID, "robi-> ", self.state)

    def get_order(self, order):
        self.orders.append(order)
        self.weight += order.weight
    
    def deliver(self, city_ID):
        if(len(self.orders) == 0):
            return
        global number_of_packages
        for idx, order in reversed(list(enumerate(self.orders))):
            if(order.destination == city_ID):
                self.weight -= order.weight
                self.orders.pop(idx)
                number_of_packages -= 1

    def changeState(self):
        if self.state == States.getting:
            return States.riding
        elif self.state == States.riding:
            return States.delivering
        else:
            return States.getting

    def printOrders(self):
        [print(order.destination, order.ID, order.weight) for order in self.orders]

    def print_info(self):
        print("Ciężarówka: ", self.ID, "w mieście -> ",self.curr_position, "robi: ", self.state)

    def wez_paczuche(self, orders):
        global cities
        for ID in orders:
            if (self.weight == self.capacity):
                break
            self.get_order(cities[self.curr_position].delete_order(ID))
            if(self.weight > self.capacity):
                cities[self.curr_position].add_order_existing(self.orders[-1])
                self.weight -= self.orders[-1].weight
                self.orders.pop()

    def pass_orders(self):
        for _ in self.orders:
            self.pass_order(self.orders.pop())
        
    def pass_order(self, order):
        cities[self.curr_position].add_order_existing(order)
        self.weight -= order.weight

    #CHOOSING ALGORITHMS
    #greedy
    def choose_orders_inorder(self):
        global cities
        IDs = []
        for order in cities[self.curr_position].orders:
            IDs.append(order.ID)
        return IDs

    #dijkstra
    def choose_orders_dijkstra(self):
        global cities
        weight = 0
        IDs = []
        if(len(cities[self.curr_position].orders) == 0):
            return IDs
        closestOrder = cities[self.curr_position].orders[0]
        closestPath = dijkstra(self.map, self.curr_position, closestOrder.destination)
        for order in cities[self.curr_position].orders:
            if (dijkstra(self.map, self.curr_position, order.destination)['distance'] < closestPath['distance']):
                closestOrder = order
        IDs.append(closestOrder.ID)
        weight += closestOrder.weight
        orders = []
        for order in cities[self.curr_position].orders:
            if order.ID == closestOrder.ID:
                continue
            orders.append({"order" : order, "distance" : dijkstra(self.map, closestOrder.destination, order.destination)['distance']})
        orders.sort(key=lambda x: x['distance'])
        for order in orders:
            if(order["order"].weight + weight > self.capacity):
                continue
            IDs.append(order["order"].ID)
        return IDs

        
    #CHOOSING DRIVING MODE
    def drive(self, destination):
        if(self.state == States.riding):
            global cities
            if(cities[self.curr_position].paths[destination] != 0):
                self.delta_t = cities[self.curr_position].paths[destination]
                self.total_distance += self.delta_t
                
                self.courses += 1
                self.curr_position = destination
            else:
                for idx, dest in enumerate(cities[self.curr_position].paths):
                    if(dest != 0):
                        self.delta_t = cities[self.curr_position].paths[idx]
                        self.total_distance += self.delta_t
                        self.courses += 1
                        self.curr_position = idx
                        break
    
    #driving_optimized
    def choose_city_dijkstra(self):
        if(len(self.orders) == 0):
            return self.choose_city_random()
        if(self.curr_position == self.orders[0].destination):
            print("err")
        path = dijkstra(self.map, self.curr_position, self.orders[0].destination)
        return path['path'][1]

    #driving_greedy  
    def choose_city_random(self):
        return random.randint(0,len(self.map)-1)


class Package:
    def __init__(self, destination, weight, package_ID):
        self.destination = destination
        self.ID = package_ID
        self.weight = weight


class City:
    def __init__(self, paths, ID):
        self.paths = paths
        self.ID = ID
        self.orders = []
        self.trucks = []
        
    def add_order(self, end, weight): #dodaj zamowienie
        global ID
        self.orders.append(Package(end, weight, ID))
        ID += 1

    def add_order_existing(self, package): #dodaj zamowienie existing
        self.orders.append(package)

    def delete_order(self,ID): #usun zamowienie
        for idx, order in enumerate(self.orders):
            if(order.ID == ID):
                return self.orders.pop(idx)

    def add_orders_random(self, number_of_orders, weight_min, weight_max):
        global number_of_packages
        number_of_packages += number_of_orders
        for _ in range(number_of_orders):
            x = random.randint(0,len(self.paths) -1)
            while(x == self.ID):
                x = random.randint(0,len(self.paths) -1)
            self.add_order(x, random.randint(weight_min, weight_max))

    
    def print_orders(self):
        print("Miasto: ", self.ID, [order.ID for order in self.orders])


class Map:
    def __init__(self, number_of_cities):
        self.paths = [[-1 for b in range(c)] for a in range(c)]

    def wypelnianko(self): 
        for a in range(c-1):
            for b in range(a + 1, c):
                temp = random.randint(1, max_distance)
                self.paths[a][b] = temp
                self.paths[b][a] = temp

    def generate_connected_graph(self, num_vertices, density, len_min, len_max):
        self.paths = [[0 for _ in range(num_vertices)] for _ in range(num_vertices)]
        for i in range(num_vertices-1):
            length = random.randint(len_min, len_max)
            self.paths[i][i+1] = length
            self.paths[i+1][i] = length
        for i in range(num_vertices):
            for j in range(i+2, num_vertices):
                if random.random() < density:
                    length = random.randint(len_min, len_max)
                    self.paths[i][j] = length
                    self.paths[j][i] = length
        return self.paths

    def print(self):
        [print(row) for row in self.paths]


def generate_trucks(trucks_number, trucks, c, capacity, map):
    for i in range(trucks_number):
        truck_ID = i
        curr_position = random.randint(0, c-1)
        trucks.append(Truck(i, curr_position, capacity, map))
        #trucks[-1].ChooseAlgorithm = Truck.choose_orders_dijkstra
        #trucks[-1].ChooseDrivingAlgorithm = Truck.choose_city_dijkstra
        trucks[-1].ChooseAlgorithm = Truck.choose_orders_inorder
        trucks[-1].ChooseDrivingAlgorithm = Truck.choose_city_random


def print_adjacency_matrix(m): # wypisywanie macierzy z odleglosciami miedzy miastami
    [print (m[a]) for a in range(len(m))]


def dijkstra(graph, start, goal):
    #initializing priority queue and adding the first vertex
    queue = [(0, start)]
    #creating a dictionary with distances from the initial vertex to the rest of the vertices 
    distances = {vertex: {'distance': float('inf'), 'path': []} for vertex in range(len(graph))}
    distances[start] = {'distance': 0, 'path': [start]}
    #creating a set with visited vertices in order to avoid a loop
    visited = set()
    #while queue is not empty
    while queue:
        #fetching the element with the shortest distance from the queue
        distance, vertex = heapq.heappop(queue)
        #if it wasn't visited, it's added to a set of visited vertices
        if vertex not in visited:
            visited.add(vertex)
            #if it's endpoint, then return distance and path to it
            if vertex == goal:
                return distances[vertex]
            #else update distances to neighbours of this vertex and add them to the queue
            for i in range(len(graph)):
                if graph[vertex][i] > 0 and distance + graph[vertex][i] < distances[i]['distance']:
                    distances[i] = {'distance': distance + graph[vertex][i], 'path': distances[vertex]['path'] + [i]}
                    heapq.heappush(queue, (distances[i]['distance'], i))


#-----MAIN-----
map = Map(c)
map.generate_connected_graph(c, 0.5, 1, 50)
#map.print()

for idx, path in enumerate(map.paths):
    cities.append(City(path, idx))
    cities[-1].add_orders_random(random.randint(150,200), 1, max_weight)
generate_trucks(trucks_number, trucks, c, capacity, map.paths)

initial_orders = number_of_packages
order_step = initial_orders/100
total = 0
courses = 0

while number_of_packages > 0:
    if(number_of_packages < initial_orders):
        initial_orders -= order_step
        print("Packages remaining to be delivered: ", number_of_packages)
    for truck in trucks:
        truck.update(1)

for truck in trucks:
    total += truck.total_distance
    courses += truck.courses

print("Total distance made by trucks: ", total, "Number of courses: ", courses)
#print_adjacency_matrix(map.paths)
print("Runtime: %s seconds" % (time.time() - time0))