from bootstrapping_olympics.utils.safe_pickle import safe_pickle_load


def load_pickle(pickle):
    return safe_pickle_load(pickle)
    
