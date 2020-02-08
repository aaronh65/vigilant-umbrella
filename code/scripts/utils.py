from MapReader import MapReader

def get_occupancy_map():
    src_path_map = '../data/map/wean.dat'
    map_obj = MapReader(src_path_map)
    occupancy_map = map_obj.get_map()
    return occupancy_map

