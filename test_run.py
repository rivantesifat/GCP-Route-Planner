from route_core import GCPRoutePlanner

print("Starting...")

planner = GCPRoutePlanner(
    "C:/Users/HDSL47/AppData/Roaming/QGIS/QGIS3/profiles/default/python/plugins/gcp_routing_tool/data/roads_V2.shp",
    "C:/Users/HDSL47/AppData/Roaming/QGIS/QGIS3/profiles/default/python/plugins/gcp_routing_tool/data/gcps_check.shp"
)

print("Loaded data")

planner.filter_roads(
    allowed_types=["primary", "residential" , "bridleway", "motorway", "busway", "primary_link", "secondary_link", "track", "secondary", "tertiary", "trunk", "trunk_link", "unclassified", "track_grade1", "track_grade1", "track_grade2", "track_grade3", "track_grade4", "track_grade4", "living_street", "steps", "service", "path", "pedestrian"],
    type_field="fclass"
)

print("Filtered roads")

planner.build_graph()
print("Graph built")

planner.snap_gcps()
planner.repair_disconnected_nodes()  # ← add this line
print("Snapped GCPs")

planner.compute_distance_matrix()
print("Distance matrix done")
print("Number of GCPs:", len(planner.gcps))
planner.cluster_gcps(max_distance_per_day=30000)
print("Clustering done")

planner.solve_routes()
print("Routing done")

planner.export_routes(
    "C:/Users/HDSL47/AppData/Roaming/QGIS/QGIS3/profiles/default/python/plugins/gcp_routing_tool/data/output_routes.shp"
)

print("Export done")
print("Done!")