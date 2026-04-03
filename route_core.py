import geopandas as gpd
import networkx as nx
from shapely.geometry import Point, LineString
from sklearn.cluster import KMeans
import numpy as np
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from scipy.spatial.distance import cdist


class GCPRoutePlanner:

    def __init__(self, road_path, gcp_path):
        self.roads = gpd.read_file(road_path)
        self.gcps = gpd.read_file(gcp_path)
        self.hotels = []        # list of snapped hotel nodes, in order [A, B]
        self.hotel_labels = []  # e.g. ["Hotel A", "Hotel B"]

    # ─────────────────────────────────────────────
    # STEP 1: Filter roads
    # ─────────────────────────────────────────────
    def filter_roads(self, allowed_types, type_field):
        self.roads = self.roads[self.roads[type_field].isin(allowed_types)]

    # ─────────────────────────────────────────────
    # STEP 2: Build graph
    # ─────────────────────────────────────────────
    def build_graph(self):
        G = nx.Graph()
        for _, row in self.roads.iterrows():
            geom = row.geometry
            lines = list(geom.geoms) if geom.geom_type == "MultiLineString" else [geom]
            for line in lines:
                coords = list(line.coords)
                for i in range(len(coords) - 1):
                    p1, p2 = coords[i], coords[i + 1]
                    G.add_edge(p1, p2, weight=Point(p1).distance(Point(p2)))
        self.G = G

    # ─────────────────────────────────────────────
    # STEP 3: Load hotels (2 inputs)
    # ─────────────────────────────────────────────
    def load_hotels(self, sources, labels=None):
        """
        sources: list of two items, each either:
                 - a shapefile path string
                 - a (x, y) coordinate tuple
        labels:  optional list of names e.g. ["Hotel A", "Hotel B"]
        """
        if labels is None:
            labels = [f"Hotel {chr(65+i)}" for i in range(len(sources))]

        self.hotels = []
        self.hotel_labels = labels
        MAX_SNAP = 500

        for i, source in enumerate(sources):
            if isinstance(source, str):
                gdf = gpd.read_file(source)
                raw_point = gdf.geometry.iloc[0]
            else:
                raw_point = Point(source)

            distances = self.roads.geometry.distance(raw_point)
            if distances.min() > MAX_SNAP:
                raise ValueError(f"{labels[i]} is too far from any road (>{MAX_SNAP}m)")

            nearest_line = self.roads.geometry.loc[distances.idxmin()]
            snapped = nearest_line.interpolate(nearest_line.project(raw_point))
            node = (snapped.x, snapped.y)

            # Connect to graph
            if node not in self.G:
                nearest = min(self.G.nodes,
                              key=lambda n: (n[0]-node[0])**2 + (n[1]-node[1])**2)
                dist = ((nearest[0]-node[0])**2 + (nearest[1]-node[1])**2)**0.5
                self.G.add_edge(node, nearest, weight=dist)

            self.hotels.append(node)
            print(f"{labels[i]} snapped to: {node}")

    # ─────────────────────────────────────────────
    # STEP 4: Snap GCPs to network
    # ─────────────────────────────────────────────
    def snap_gcps(self):
        road_geoms = self.roads.geometry
        MAX_SNAP = 500
        snapped_nodes = []

        for _, row in self.gcps.iterrows():
            point = row.geometry
            distances = road_geoms.distance(point)
            if distances.min() > MAX_SNAP:
                print(f"⚠️ GCP too far from road: {point}")
                snapped_nodes.append(None)
                continue
            nearest_line = road_geoms.loc[distances.idxmin()]
            snapped = nearest_line.interpolate(nearest_line.project(point))
            snapped_nodes.append((snapped.x, snapped.y))

        self.gcps["node"] = snapped_nodes
        self.gcps = self.gcps[self.gcps["node"].notnull()].copy()
        self.gcps = self.gcps.reset_index(drop=True)

        for node in self.gcps["node"]:
            if node not in self.G:
                nearest = min(self.G.nodes,
                              key=lambda n: (n[0]-node[0])**2 + (n[1]-node[1])**2)
                dist = ((nearest[0]-node[0])**2 + (nearest[1]-node[1])**2)**0.5
                self.G.add_edge(node, nearest, weight=dist)

    def repair_disconnected_nodes(self):
        largest_cc = max(nx.connected_components(self.G), key=len)
        repaired = 0
        for node in self.gcps["node"]:
            if node not in largest_cc:
                nearest = min(largest_cc,
                              key=lambda n: (n[0]-node[0])**2 + (n[1]-node[1])**2)
                dist = ((nearest[0]-node[0])**2 + (nearest[1]-node[1])**2)**0.5
                self.G.add_edge(node, nearest, weight=dist)
                repaired += 1
        print(f"Repaired {repaired} disconnected GCP nodes")

    # ─────────────────────────────────────────────
    # STEP 5: Split GCPs between hotels
    # by road distance (falls back to straight-line
    # if path not found)
    # ─────────────────────────────────────────────
    def assign_gcps_to_hotels(self):
        """
        Each GCP is assigned to whichever hotel is
        closer by road network distance.
        Stores result in self.gcps["hotel_idx"] (0 or 1).
        """
        assignments = []

        for _, row in self.gcps.iterrows():
            node = row["node"]
            dists = []
            for hotel_node in self.hotels:
                try:
                    d = nx.shortest_path_length(
                        self.G, hotel_node, node, weight="weight")
                except (nx.NetworkXNoPath, nx.NodeNotFound):
                    # fallback to straight-line
                    d = ((hotel_node[0]-node[0])**2 +
                         (hotel_node[1]-node[1])**2)**0.5
                dists.append(d)
            assignments.append(int(np.argmin(dists)))

        self.gcps["hotel_idx"] = assignments

        for i, label in enumerate(self.hotel_labels):
            count = (self.gcps["hotel_idx"] == i).sum()
            print(f"{label}: {count} GCPs assigned")

    # ─────────────────────────────────────────────
    # STEP 6: Distance matrix (full, for all GCPs)
    # ─────────────────────────────────────────────
    def compute_distance_matrix(self):
        nodes = self.gcps["node"].tolist()
        n = len(nodes)
        matrix = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                if i == j:
                    continue
                try:
                    matrix[i][j] = nx.shortest_path_length(
                        self.G, nodes[i], nodes[j], weight="weight")
                except (nx.NetworkXNoPath, nx.NodeNotFound):
                    matrix[i][j] = 1e6
        self.distance_matrix = matrix
        print("Distance matrix computed")

    # ─────────────────────────────────────────────
    # STEP 7: Cluster per hotel zone independently
    # ─────────────────────────────────────────────
    def cluster_gcps(self, max_distance_per_day):
        self.gcps["day"] = -1  # will be filled per hotel

        global_day_counter = 0

        for hotel_idx, hotel_node in enumerate(self.hotels):
            label = self.hotel_labels[hotel_idx]
            subset = self.gcps[self.gcps["hotel_idx"] == hotel_idx].copy()

            if len(subset) == 0:
                print(f"{label}: no GCPs assigned, skipping")
                continue

            coords = np.array([(p.x, p.y) for p in subset.geometry])

            # Estimate k from average spatial distance
            if len(subset) < 2:
                k = 1
            else:
                spatial_dists = cdist(coords, coords)
                np.fill_diagonal(spatial_dists, np.nan)
                avg_dist = np.nanmean(spatial_dists)
                gcps_per_day = max(2, int(max_distance_per_day / avg_dist))
                k = max(2, int(np.ceil(len(subset) / gcps_per_day)))
                k = min(k, len(subset) // 2)

            print(f"{label}: {len(subset)} GCPs → {k} day clusters")

            kmeans = KMeans(n_clusters=k, random_state=0, n_init=10).fit(coords)
            local_labels = kmeans.labels_

            # Map local cluster labels to global day numbers
            # Order clusters by distance from hotel (farthest first)
            centroids = kmeans.cluster_centers_
            hotel_pt = np.array(hotel_node)
            centroid_dists = [
                np.linalg.norm(centroids[c] - hotel_pt)
                for c in range(k)
            ]
            # farthest first → reversed sort
            order = np.argsort(centroid_dists)[::-1]
            local_to_global = {}
            for rank, cluster_id in enumerate(order):
                local_to_global[cluster_id] = global_day_counter + rank

            for idx, (df_idx, _) in enumerate(subset.iterrows()):
                self.gcps.at[df_idx, "day"] = local_to_global[local_labels[idx]]

            global_day_counter += k

        # Fix small clusters across the whole GCP set
        self._fix_small_clusters()
        self._renumber_days()

    # ─────────────────────────────────────────────
    # STEP 7b: Fix tiny clusters
    # ─────────────────────────────────────────────
    def _fix_small_clusters(self):
        for _ in range(10):
            counts = self.gcps["day"].value_counts()
            small_days = counts[counts < 2].index
            if len(small_days) == 0:
                break
            for day in small_days:
                idx = self.gcps[self.gcps["day"] == day].index
                for i in idx:
                    best_day, best_dist = None, np.inf
                    others = self.gcps[self.gcps["day"] != day]
                    for other_day in others["day"].unique():
                        for j in others[others["day"] == other_day].index:
                            d = self.distance_matrix[i][j]
                            if d < best_dist:
                                best_dist = d
                                best_day = other_day
                    if best_day is not None:
                        self.gcps.at[i, "day"] = best_day

    def _renumber_days(self):
        label_map = {old: new for new, old in
                     enumerate(sorted(self.gcps["day"].unique()))}
        self.gcps["day"] = self.gcps["day"].map(label_map)
        print(f"Days renumbered: 0 to {len(label_map)-1}")

    # ─────────────────────────────────────────────
    # STEP 8: Solve TSP per day
    # ─────────────────────────────────────────────
    def solve_routes(self):
        routes = []

        for day in sorted(self.gcps["day"].unique()):
            subset = self.gcps[self.gcps["day"] == day]
            indices = subset.index.tolist()

            # Which hotel does this day belong to?
            hotel_votes = subset["hotel_idx"].value_counts()
            hotel_idx = hotel_votes.idxmax()
            hotel_node = self.hotels[hotel_idx]

            # Build sub-matrix including hotel as node 0
            nodes_for_day = [hotel_node] + \
                            [self.gcps.loc[i]["node"] for i in indices]
            n = len(nodes_for_day)
            sub_matrix = np.zeros((n, n))

            for a in range(n):
                for b in range(n):
                    if a == b:
                        continue
                    try:
                        sub_matrix[a][b] = nx.shortest_path_length(
                            self.G, nodes_for_day[a],
                            nodes_for_day[b], weight="weight")
                    except (nx.NetworkXNoPath, nx.NodeNotFound):
                        sub_matrix[a][b] = 1e6

            # OR-Tools: depot = index 0 (hotel), open route
            # (hotel → GCPs → hotel is handled in export)
            manager = pywrapcp.RoutingIndexManager(n, 1, 0)
            routing = pywrapcp.RoutingModel(manager)

            def distance_callback(from_index, to_index, m=sub_matrix):
                return int(m[manager.IndexToNode(from_index)]
                             [manager.IndexToNode(to_index)])

            cb_idx = routing.RegisterTransitCallback(distance_callback)
            routing.SetArcCostEvaluatorOfAllVehicles(cb_idx)

            params = pywrapcp.DefaultRoutingSearchParameters()
            params.first_solution_strategy = (
                routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

            solution = routing.SolveWithParameters(params)

            route = []
            if solution:
                index = routing.Start(0)
                while not routing.IsEnd(index):
                    node_pos = manager.IndexToNode(index)
                    if node_pos > 0:  # skip hotel (node 0)
                        route.append(indices[node_pos - 1])
                    index = solution.Value(routing.NextVar(index))

            routes.append((day, hotel_idx, route))

        self.routes = routes

    # ─────────────────────────────────────────────
    # STEP 9: Export — full round trips
    # ─────────────────────────────────────────────
    def export_routes(self, output_path):
        lines = []

        for day, hotel_idx, route in self.routes:
            if len(route) < 1:
                print(f"Skipping day {day} (empty route)")
                continue

            hotel_node = self.hotels[hotel_idx]
            hotel_label = self.hotel_labels[hotel_idx]
            full_coords = []

            # Hotel → first GCP
            try:
                path = nx.shortest_path(
                    self.G, hotel_node,
                    self.gcps.loc[route[0]]["node"], weight="weight")
                full_coords.extend(path)
            except (nx.NetworkXNoPath, nx.NodeNotFound):
                print(f"Day {day}: path from hotel to first GCP failed")

            # GCP → GCP
            for i in range(len(route) - 1):
                n1 = self.gcps.loc[route[i]]["node"]
                n2 = self.gcps.loc[route[i + 1]]["node"]
                try:
                    path = nx.shortest_path(self.G, n1, n2, weight="weight")
                    full_coords.extend(path[1:])
                except (nx.NetworkXNoPath, nx.NodeNotFound):
                    print(f"Day {day}: path failed {n1} → {n2}")

            # Last GCP → Hotel (return trip)
            try:
                path = nx.shortest_path(
                    self.G, self.gcps.loc[route[-1]]["node"],
                    hotel_node, weight="weight")
                full_coords.extend(path[1:])
            except (nx.NetworkXNoPath, nx.NodeNotFound):
                print(f"Day {day}: return path to hotel failed")

            if len(full_coords) < 2:
                continue

            total_dist_km = sum(
                ((full_coords[i][0] - full_coords[i-1][0])**2 +
                 (full_coords[i][1] - full_coords[i-1][1])**2)**0.5
                for i in range(1, len(full_coords))
            ) / 1000

            lines.append({
                "day": int(day),
                "hotel": hotel_label,
                "round_trip_km": round(total_dist_km, 2),
                "geometry": LineString(full_coords)
            })

        if not lines:
            print("No valid routes to export!")
            return

        gdf = gpd.GeoDataFrame(lines, crs=self.gcps.crs)
        gdf.to_file(output_path)
        print(f"Exported {len(lines)} routes")
        print(gdf[["day", "hotel", "round_trip_km"]].to_string())
