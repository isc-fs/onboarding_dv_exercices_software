from track_generator import TrackGenerator
from utils import Mode, SimType

# Input parameters
n_points = 70
n_regions = 40
min_bound = 10.
max_bound = 200.
mode = Mode.RANDOM
sim_type = SimType.FSSIM

# Output options
plot_track = True
visualise_voronoi = True
create_output_file = True
output_location = '/tracks'

# Generate track
track_gen = TrackGenerator(n_points, n_regions, min_bound, max_bound, mode, plot_track, visualise_voronoi, create_output_file, output_location, lat_offset=51.197682, lon_offset=5.323411, sim_type=sim_type)
track_gen.create_track()
