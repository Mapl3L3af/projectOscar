import openrouteservice
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import numpy as np

# Initialize the OpenRouteService client with your API key
client = openrouteservice.Client(key='5b3ce3597851110001cf62481bfd93f100af4c96ab9e80abf218a1c8')

# Function to geocode addresses and get coordinates
def geocode_address(address):
    try:
        result = client.pelias_search(text=address)
        if result and 'features' in result and len(result['features']) > 0:
            coords = result['features'][0]['geometry']['coordinates']
            return coords[1], coords[0]  # Return (latitude, longitude)
        else:
            print(f"Address '{address}' could not be geocoded.")
            return None
    except Exception as e:
        print(f"Error geocoding address '{address}': {e}")
        return None

# Read addresses from the input file and strip newlines
addresses = []
with open('la_routes.txt', 'r') as file:
    addresses = [line.strip() for line in file if line.strip()]  # Ensure no empty lines are processed

# Geocode the addresses to get coordinates
coordinates = [geocode_address(address) for address in addresses if geocode_address(address)]
if not coordinates:
    print("No valid coordinates found.")
    exit()

# Function to create a distance matrix using batch requests
def create_distance_matrix(coords, batch_size=10):
    num_locations = len(coords)
    matrix = np.zeros((num_locations, num_locations), dtype=int)

    # Split coordinates into manageable batches to stay within API call limits
    for i in range(0, num_locations, batch_size):
        for j in range(0, num_locations, batch_size):
            sub_coords = coords[i:i + batch_size]
            sub_matrix_coords = coords[j:j + batch_size]

            response = client.distance_matrix(
                locations=[(coord[1], coord[0]) for coord in (sub_coords + sub_matrix_coords)],
                profile='driving-car',
                metrics=['distance']
            )

            if 'distances' in response:
                distances = response['distances']
                for m, row in enumerate(distances[:len(sub_coords)]):
                    matrix[i + m, j:j + len(sub_matrix_coords)] = row[:len(sub_matrix_coords)]
            else:
                print("Error retrieving the distance matrix.")
                exit()

    return matrix

# Create the distance matrix from the geocoded coordinates
distance_matrix = create_distance_matrix(coordinates)

# Function to calculate the optimal number of vehicles based on the number of locations
def get_optimal_num_vehicles(num_locations, max_locations_per_vehicle=5):
    # Estimate the number of vehicles needed
    return max(1, (num_locations + max_locations_per_vehicle - 1) // max_locations_per_vehicle)

# Solve the VRP using Google OR-Tools
def solve_vrp(distance_matrix, num_vehicles, depot=0):
    num_locations = len(distance_matrix)
    manager = pywrapcp.RoutingIndexManager(num_locations, num_vehicles, depot)
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(distance_matrix[from_node][to_node])

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add distance constraint to ensure vehicles return to the depot
    dimension_name = 'Distance'
    routing.AddDimension(
        transit_callback_index,
        0,  # no slack
        32187,  # vehicle maximum travel distance (adjust as needed)
        True,  # start cumul to zero
        dimension_name
    )
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Define search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)
    if solution:
        with open('vehicle_routes_output.txt', 'w') as output_file:
            for vehicle_id in range(num_vehicles):
                index = routing.Start(vehicle_id)
                route_coordinates = []
                route_distance = 0  # Initialize the total distance for the route

                while not routing.IsEnd(index):
                    node_index = manager.IndexToNode(index)
                    route_coordinates.append(coordinates[node_index])  # Append (lat, lon) coordinates
                    previous_index = index
                    index = solution.Value(routing.NextVar(index))
                    route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

                route_coordinates.append(coordinates[depot])  # Append depot coordinates at the end

                # Write route to the file with total distance
                output_file.write(f"Route {vehicle_id + 1}: {route_distance} m\n")
                for coord in route_coordinates:
                    output_file.write(f"{coord[0]}, {coord[1]}\n")  # Write latitude, longitude format
                output_file.write("\n")

                # Print route to the console (optional)
                print(f"Route for vehicle {vehicle_id + 1}: {route_distance} m")
                for coord in route_coordinates:
                    print(f"{coord[0]}, {coord[1]} -> ", end="")
                print("\n")
    else:
        print("No solution found!")

# Determine the number of vehicles based on the number of addresses
num_vehicles = get_optimal_num_vehicles(len(coordinates))

# Run the solver with the dynamically determined number of vehicles
solve_vrp(distance_matrix, num_vehicles)
