from math import radians, sqrt, sin, cos, atan2
from random import random


class Heuristic:
    def __init__(self):
        self.city_coordinate_map: dict[str, tuple[float, float]] = {}
        self.random_node_coordinate_map = {}
        # let's read the cities, latitudes and longitudes from the file
        with open('romania.txt', 'r') as file:
            while True:
                line = file.readline()
                if (line == ""):
                    break
                # we use 4 spaces to separate the cities and numbers
                unpacked = line.split(sep="    ")
                city, lat, long = unpacked
                self.city_coordinate_map[city] = (float(lat), float(long))

        # now let's populate the random coordinates map
        for i in range(50):
            self.random_node_coordinate_map[str(i)] = (random()* 10, random() * 10)

    def distance(self, coordinates_1, coordinates_2):

        # Convert latitudes and longitudes from degrees to radians
        latitude_1_rad, longitude_1_rad = radians(
            coordinates_1[0]), radians(coordinates_1[1])
        latitude_2_rad, longitude_2_rad = radians(
            coordinates_2[0]), radians(coordinates_2[1])

        # Haversine formula
        distance_latitude = latitude_2_rad - latitude_1_rad
        distance_longitude = longitude_2_rad - longitude_1_rad
        a = sin(distance_latitude / 2)**2 + cos(latitude_1_rad) * \
            cos(latitude_2_rad) * sin(distance_longitude / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1-a))

        # Earth's radius in kilometers
        R = 6371

        # Distance in kilometers
        distance = R * c

        return distance
    
    def romaniaDistance(self, node, goal):

        latitude_1, longitude_1 = self.city_coordinate_map[node]
        latitude_2, longitude_2 = self.city_coordinate_map[goal]

        return self.distance((latitude_1, longitude_1), (latitude_2, longitude_2))
    

    
    def randomDistance(self, node1, node2):
        return self.distance(self.random_node_coordinate_map[node1], self.random_node_coordinate_map[node2])
    

