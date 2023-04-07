from math import radians, sqrt, sin, cos, atan2


class Heuristic:
    def __init__(self):
        self.city_coordinate_map: dict[str, tuple[float, float]] = {}
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

    def distance(self, node, goal):

        latitude_1, longitude_1 = self.city_coordinate_map[node]
        latitude_2, longitude_2 = self.city_coordinate_map[goal]

        # Convert latitudes and longitudes from degrees to radians
        latitude_1_rad, longitude_1_rad = radians(
            latitude_1), radians(longitude_1)
        latitude_2_rad, longitude_2_rad = radians(
            latitude_2), radians(longitude_2)

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
