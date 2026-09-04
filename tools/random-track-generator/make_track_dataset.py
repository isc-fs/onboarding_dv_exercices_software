from track_generator import TrackGenerator
from utils import Mode, SimType
from datetime import datetime

NUM_TRACKS = 20

# Input parameters for track
n_points = 70
n_regions = 40
min_bound = 10.
max_bound = 200.
mode = Mode.RANDOM
sim_type = SimType.FSSIM

# Output options 
plot_track = False
visualise_voronoi = False
create_output_file = True
output_location = f'/track_dataset_{datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}'

# Generate track
track_gen = TrackGenerator(n_points, n_regions, min_bound, max_bound, mode, plot_track, visualise_voronoi, create_output_file, output_location, lat_offset=51.197682, lon_offset=5.323411, sim_type=sim_type)

for i in range(NUM_TRACKS):
    track_gen.create_track(f'track_{i}')

print(f'Created {NUM_TRACKS} tracks in {output_location}')
