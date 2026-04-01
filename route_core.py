import geopandas as gpd
import networkx as nx
from shapely.geometry import Point, LineString
from sklearn.cluster import KMeans
import numpy as np
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from scipy.spatial import KDTree


class GCPRoutePlanner:

    def __init__(self, road_path, gcp_path):
        self.roads = gpd.read_file(road_path)
        self.gcps = gpd.read_file(gcp_path)

    # -----------------------------
    # STEP 1: Filter road types
    # -----------------------------
    def filter_roads(self, allowed_types, type_field):
        self.roads = self.roads[self.roads[type_field].isin(allowed_types)]

    # -----------------------------
    # STEP 2: Build Graph
    # -----------------------------
    def build_graph(self):
        G = nx.Graph()

        for _, row in self.roads.iterrows():
            geom = row.geometry

            if geom.geom_type == "LineString":
                lines = [geom]
            elif geom.geom_type == "MultiLineString":
                lines = list(geom.geoms)
            else:
                continue

            for line in lines:
                coords = list(line.coords)
                for i in range(len(coords) - 1):
                    p1 = coords[i]
                    p2 = coords[i + 1]
                    dist = Point(p1).distance(Point(p2))
                    G.add_edge(p1, p2, weight=dist)

        self.G = G

    # -----------------------------
    # STEP 3: Snap GCPs to network
    # -----------------------------
    def snap_gcps(self):
        road_geoms = self.roads.geometry
        MAX_SNAP_DIST = 500

        snapped_nodes = []

        for _, row in self.gcps.iterrows():
            point = row.geometry

            distances = road_geoms.distance(point)
            min_dist = distances.min()

            if min_dist > MAX_SNAP_DIST:
                print(f"⚠️ GCP too far from road: {point}")
                snapped_nodes.append(None)
                continue

            nearest_idx = distances.idxmin()
            nearest_line = road_geoms.loc[nearest_idx]

            snapped_point = nearest_line.interpolate(
                nearest_line.project(point)
            )

            snapped_nodes.append((snapped_point.x, snapped_point.y))

        # Assign and clean
        self.gcps["node"] = snapped_nodes
        self.gcps = self.gcps[self.gcps["node"].notnull()].copy()
        self.gcps = self.gcps.reset_index(drop=True)  # FIX 1: reset index to 0-based

        # Connect snapped nodes to graph
        for node in self.gcps["node"]:
            if node not in self.G:
                nearest = min(
                    self.G.nodes,
                    key=lambda n: (n[0] - node[0]) ** 2 + (n[1] - node[1]) ** 2
                )
                dist = ((nearest[0] - node[0]) ** 2 + (nearest[1] - node[1]) ** 2) ** 0.5
                self.G.add_edge(node, nearest, weight=dist)

    def repair_disconnected_nodes(self):
        # Find the largest connected component
        largest_cc = max(nx.connected_components(self.G), key=len)
        subgraph_nodes = set(largest_cc)

        repaired = 0
        for node in self.gcps["node"]:
            if node not in subgraph_nodes:
                # Force-connect to nearest node IN the main component
                nearest = min(
                    subgraph_nodes,
                    key=lambda n: (n[0] - node[0]) ** 2 + (n[1] - node[1]) ** 2
                )
                dist = ((nearest[0] - node[0]) ** 2 + (nearest[1] - node[1]) ** 2) ** 0.5
                self.G.add_edge(node, nearest, weight=dist)
                print(f"Repaired disconnected node: {node} → connected via {dist:.0f}m bridge")
                repaired += 1

        print(f"Repaired {repaired} disconnected GCP nodes")

    # -----------------------------
    # STEP 4: Distance Matrix
    # -----------------------------
    def compute_distance_matrix(self):
        nodes = self.gcps["node"].tolist()
        n = len(nodes)
        matrix = np.zeros((n, n))

        for i in range(n):
            for j in range(n):
                if i == j:
                    continue
                try:
                    length = nx.shortest_path_length(
                        self.G,
                        nodes[i],
                        nodes[j],
                        weight="weight"
                    )
                except (nx.NetworkXNoPath, nx.NodeNotFound):  # FIX: specific exception
                    length = 1e6
                matrix[i][j] = length

        self.distance_matrix = matrix

    # -----------------------------
    # STEP 5: Clustering (Daily groups)
    # -----------------------------
    def cluster_gcps(self, max_distance_per_day):
        coords = np.array([(p.x, p.y) for p in self.gcps.geometry])

        # Use straight-line distances between GCPs to estimate k
        # (network distances are too inflated for this estimate)
        from scipy.spatial.distance import cdist
        spatial_dists = cdist(coords, coords)
        np.fill_diagonal(spatial_dists, np.nan)
        avg_spatial_dist = np.nanmean(spatial_dists)

        gcps_per_day = max(2, int(max_distance_per_day / avg_spatial_dist))
        k = max(2, int(np.ceil(len(self.gcps) / gcps_per_day)))
        k = min(k, len(self.gcps) // 2)

        print(f"Avg straight-line GCP distance: {avg_spatial_dist:.0f}m")
        print(f"GCPs per day: {gcps_per_day}, Days: {k}")

        kmeans = KMeans(n_clusters=k, random_state=0, n_init=10).fit(coords)
        self.gcps["day"] = kmeans.labels_
        self.fix_small_clusters()

    # -----------------------------
    # STEP 5b: Fix tiny clusters
    # -----------------------------
    def fix_small_clusters(self):
        # Run multiple passes until no small clusters remain
        for _ in range(10):  # max 10 passes
            counts = self.gcps["day"].value_counts()
            small_days = counts[counts < 2].index
            if len(small_days) == 0:
                break

            for day in small_days:
                idx = self.gcps[self.gcps["day"] == day].index
                for i in idx:
                    current_point = self.gcps.loc[i].geometry
                    others = self.gcps[self.gcps["day"] != day]
                    if len(others) == 0:
                        continue

                    # Use network distance if available, else spatial
                    best_day = None
                    best_dist = np.inf
                    for other_day in others["day"].unique():
                        day_gcps = others[others["day"] == other_day]
                        for j in day_gcps.index:
                            d = self.distance_matrix[i][j]
                            if d < best_dist:
                                best_dist = d
                                best_day = other_day

                    if best_day is not None:
                        self.gcps.at[i, "day"] = best_day
                        print(f"  Reassigned isolated GCP {i} → day {best_day} (dist: {best_dist:.0f}m)")

    # -----------------------------
    # STEP 6: Solve TSP per cluster
    # -----------------------------
    def solve_routes(self):
        routes = []

        for day in self.gcps["day"].unique():
            subset = self.gcps[self.gcps["day"] == day]
            indices = subset.index.tolist()  # 0-based after reset_index in snap_gcps
            sub_matrix = self.distance_matrix[np.ix_(indices, indices)]

            manager = pywrapcp.RoutingIndexManager(len(sub_matrix), 1, 0)
            routing = pywrapcp.RoutingModel(manager)

            # FIX 2: capture sub_matrix by value via default arg to avoid closure bug
            def distance_callback(from_index, to_index, m=sub_matrix):
                f = manager.IndexToNode(from_index)
                t = manager.IndexToNode(to_index)
                return int(m[f][t])

            transit_callback_index = routing.RegisterTransitCallback(distance_callback)
            routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

            search_params = pywrapcp.DefaultRoutingSearchParameters()
            search_params.first_solution_strategy = (
                routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
            )

            solution = routing.SolveWithParameters(search_params)

            route = []
            if solution:
                index = routing.Start(0)
                while not routing.IsEnd(index):
                    node = manager.IndexToNode(index)
                    route.append(indices[node])
                    index = solution.Value(routing.NextVar(index))

            routes.append((day, route))

        self.routes = routes

    # -----------------------------
    # STEP 7: Export
    # -----------------------------
    def export_routes(self, output_path):
        lines = []

        for day, route in self.routes:
            if len(route) < 2:
                print(f"Skipping day {day} (only {len(route)} point)")
                continue

            full_path_coords = []

            for i in range(len(route) - 1):
                n1 = self.gcps.loc[route[i]]["node"]
                n2 = self.gcps.loc[route[i + 1]]["node"]

                try:
                    path = nx.shortest_path(self.G, n1, n2, weight="weight")
                    if i > 0:
                        path = path[1:]  # avoid duplicate junction points
                    full_path_coords.extend(path)
                except (nx.NetworkXNoPath, nx.NodeNotFound):  # FIX: specific exception
                    print(f"Path failed between {n1} and {n2}")

            if len(full_path_coords) < 2:
                continue

            line = LineString(full_path_coords)
            lines.append({
                "day": int(day),
                "geometry": line
            })

        if len(lines) == 0:
            print("No valid routes to export!")
            return

        gdf = gpd.GeoDataFrame(lines, crs=self.gcps.crs)
        gdf.to_file(output_path)
        print("Routes exported:", len(lines))

    # -----------------------------
    # UTILITY: Check connectivity
    # -----------------------------
    def check_connectivity(self):  # FIX 3: now correctly inside the class
        nodes = self.gcps["node"].tolist()
        for i in range(len(nodes) - 1):
            try:
                nx.shortest_path(self.G, nodes[i], nodes[i + 1])
            except (nx.NetworkXNoPath, nx.NodeNotFound):  # FIX: syntax was broken before
                print(f"Disconnected: {nodes[i]} → {nodes[i + 1]}")