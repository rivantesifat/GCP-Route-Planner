# GCP Route Planner

A road-network-aware routing tool for **Ground Control Point (GCP) survey field planning**, used in active photogrammetric survey operations. Given a set of GCP locations, a road network, and one or two base hotels, the tool automatically clusters GCPs into daily survey routes and solves an optimal visitation sequence for each day using the Travelling Salesman Problem (TSP).

---

## How It Works

The pipeline runs in nine sequential steps:

```
Road network (.shp)  ──┐
GCP locations (.shp)  ──┤──► Filter roads ──► Build graph ──► Snap GCPs/Hotels
Hotel locations (.shp) ──┘
                                                                      │
                              ┌───────────────────────────────────────┘
                              ▼
                    Assign GCPs to hotels ──► Compute distance matrix
                              │
                              ▼
                    Cluster GCPs into days (K-Means + distance budget)
                              │
                              ▼
                    Solve TSP per day (OR-Tools)
                              │
                              ▼
                    Export road-following routes (.shp)
```

1. **Filter roads** — retains only traversable road types (primary, secondary, track, path, etc.)
2. **Build graph** — constructs a weighted NetworkX graph from road segment vertices
3. **Load hotels** — accepts shapefile paths or raw coordinates; snaps to nearest road node
4. **Snap GCPs** — projects each GCP onto the nearest road geometry; discards GCPs > 500 m from any road
5. **Repair disconnected nodes** — reconnects isolated graph components to the largest connected component
6. **Assign GCPs to hotels** — each GCP is assigned to the closer hotel by road network distance (falls back to straight-line if no path exists)
7. **Cluster GCPs** — K-Means clustering per hotel zone, with cluster count estimated from a daily distance budget; farthest clusters scheduled first; tiny clusters dissolved by proximity
8. **Solve routes** — OR-Tools TSP solver (`PATH_CHEAPEST_ARC`) per day, with hotel as depot
9. **Export routes** — road-following polylines exported as a shapefile with `day`, `hotel`, and `round_trip_km` attributes

---
## Sample Output

The map below shows a real routing output for a GCP survey campaign in 
Sri Lanka — 20 survey days across two hotel bases, with road-following 
round trips exported directly to QGIS.

![Sample routing output — Sri Lanka GCP survey campaign](srilanka.png)

*20 survey days (Days 0–19) optimised across Hotel A and Hotel B. 
Each colour represents one day's road-following round trip. 
Red diamonds indicate GCP locations.*
```
## Installation

```bash
pip install geopandas networkx shapely scikit-learn ortools scipy
```

> **Note:** If using inside QGIS, these libraries must be installed into QGIS's Python environment. Use the OSGeo4W shell or QGIS's built-in Python console with `pip`.

---

## Usage

### As a standalone Python script

```python
from route_core import GCPRoutePlanner

planner = GCPRoutePlanner(
    road_path="data/roads.shp",
    gcp_path="data/gcps.shp"
)

planner.filter_roads(
    allowed_types=[
        "primary", "secondary", "tertiary", "trunk", "trunk_link",
        "primary_link", "secondary_link", "residential", "unclassified",
        "living_street", "service", "track", "track_grade1", "track_grade2",
        "track_grade3", "track_grade4", "path", "pedestrian", "steps",
        "bridleway", "busway", "motorway"
    ],
    type_field="fclass"
)

planner.build_graph()

planner.load_hotels(
    sources=["data/hotels/hotel_a.shp", "data/hotels/hotel_b.shp"],
    labels=["Hotel A", "Hotel B"]
)

planner.snap_gcps()
planner.repair_disconnected_nodes()
planner.compute_distance_matrix()

planner.assign_gcps_to_hotels()
planner.cluster_gcps(max_distance_per_day=30000)  # 30 km daily budget

planner.solve_routes()
planner.export_routes("data/results/output_routes.shp")
```

### Parameters

| Parameter | Description |
|---|---|
| `road_path` | Path to road network shapefile |
| `gcp_path` | Path to GCP point shapefile |
| `allowed_types` | List of road type strings to retain (matched against `type_field`) |
| `type_field` | Attribute field name in the road shapefile containing road type (e.g. `"fclass"`) |
| `sources` | List of hotel shapefiles or `(x, y)` coordinate tuples |
| `labels` | Display names for each hotel (e.g. `["Hotel A", "Hotel B"]`) |
| `max_distance_per_day` | Maximum daily travel distance in metres (used to estimate cluster count) |
| `output_path` | Output shapefile path for exported routes |

---

## Output

The exported shapefile contains one feature per survey day with the following attributes:

| Field | Type | Description |
|---|---|---|
| `day` | Integer | Survey day number (0-indexed) |
| `hotel` | String | Base hotel for that day |
| `round_trip_km` | Float | Total road-following round trip distance in kilometres |
| `geometry` | LineString | Full road-following route (hotel → GCPs → hotel) |

Load directly into QGIS for field navigation.

---

## Project Structure

```
gcp-route-planner/
│
├── route_core.py          # GCPRoutePlanner class — full pipeline
├── test_run.py            # Example usage script
├── data/
│   ├── roads.shp          # Road network (OSM or equivalent)
│   ├── gcps.shp           # GCP point locations
│   ├── hotels/
│   │   ├── hotel_a.shp
│   │   └── hotel_b.shp
│   └── results/
│       └── output_routes.shp
└── README.md
```

---

## Key Design Decisions

**Road-network snapping** — GCPs and hotels are snapped to the nearest point on the road geometry using Shapely's `project`/`interpolate` pattern, rather than to the nearest graph node. This avoids large position errors in areas with long road segments.

**Connectivity repair** — After snapping, any GCP node not in the largest connected component is stitched to its nearest node in that component. This prevents `NetworkXNoPath` failures in fragmented OSM networks.

**Hotel-first assignment** — GCPs are partitioned between hotels by road distance before clustering. This means each hotel's cluster is spatially coherent and avoids cross-hotel routing inefficiencies.

**Daily budget estimation** — The number of K-Means clusters (survey days) is estimated from the ratio of `max_distance_per_day` to the average pairwise spatial distance among GCPs in each hotel zone, with a minimum of 2 clusters.

**Farthest-first scheduling** — Within each hotel's cluster set, the cluster whose centroid is farthest from the hotel is scheduled first. This is a heuristic to front-load difficult travel days.

---

## Dependencies

| Library | Purpose |
|---|---|
| `geopandas` | Shapefile I/O and spatial operations |
| `networkx` | Road graph construction and shortest paths (Dijkstra) |
| `shapely` | Geometry snapping (`project` / `interpolate`) |
| `scikit-learn` | K-Means clustering |
| `ortools` | TSP solver (`PATH_CHEAPEST_ARC` strategy) |
| `scipy` | Pairwise distance matrix for cluster estimation |
| `numpy` | Array operations |

---

## Limitations

- Road network must be in a **projected CRS** (metres) — geographic CRS (degrees) will produce incorrect distance calculations
- Distance matrix computation is O(N²) via Dijkstra — for very large GCP sets (500+) this will be slow
- TSP solver uses a greedy construction heuristic (`PATH_CHEAPEST_ARC`); solutions are near-optimal but not guaranteed optimal for large day clusters
- Hotels must be within 500 m of a road segment to be accepted

---

## License

MIT License. See `LICENSE` for details.

---

## Author

**Md. Nashid Kamal Sifat**  
GEO-ICT Engineer | HawarIT, Netherlands  
[GitHub](https://github.com/rivantesifat) · [LinkedIn](https://linkedin.com/in/nashidsifat)
