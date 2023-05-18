r_graph = {

    'B0101': {'B0102': 90, 'B0201': 0},
    'B0102': {'B0101': -90, 'B0107': 0, 'B0103': 90},
    'B0103': {'B0102': -90, 'B0108': 0, 'B0104': 90},
    'B0104': {'B0103': -90, 'B0109': 0, 'B0105': 90},
    'B0105': {'B0104': -90, 'B0110': 0, 'B0106': 90},
    'B0106': {'B0105': -90, 'B0206': 0},

    'B0201': {'B0101': 180, 'B0202': 90, 'B0301': 0},
    'B0202': {'B0203': 90, 'B0107': 180, 'B0302': 0, 'B0201': -90},
    'B0203': {'B0204': 90, 'B0108': 180, 'B0303': 0, 'B0202': -90},
    'B0204': {'B0205': 90, 'B0109': 180, 'B0304': 0, 'B0203': -90},
    'B0205': {'B0206': 90, 'B0110': 180, 'B0305': 0, 'B0204': -90},
    'B0206': {'B0105': 180, 'B0305': 0, 'B0204': -90},

    'B0301': {'B0201': 180, 'B0302': 90, 'B0401': 0},
    'B0302': {'B0202': 180, 'B0111': 0, 'B0303': 90, 'B0301': -90},
    'B0303': {'B0203': 180, 'B0112': 0, 'B0304': 90, 'B0302': -90},
    'B0304': {'B0204': 180, 'B0113': 0, 'B0305': 90, 'B0303': -90},
    'B0305': {'B0205': 180, 'B0114': 0, 'B0306': 90, 'B0304': -90},
    'B0306': {'B0206': 180, 'B0406': 0, 'B0305': -90},

    'B0401': {'B0301': 180, 'B0402': 90, 'B0501': 0},
    'B0402': {'B0403': 90, 'B0111': 180, 'B0502': 0, 'B0401': -90},
    'B0403': {'B0404': 90, 'B0112': 180, 'B0503': 0, 'B0402': -90},
    'B0404': {'B0405': 90, 'B0113': 180, 'B0504': 0, 'B0403': -90},
    'B0405': {'B0406': 90, 'B0114': 180, 'B0505': 0, 'B0404': -90},
    'B0406': {'B0306': 180, 'B0506': 0, 'B0405': -90},

    'B0501': {'B0401': 180, 'B0502': 90},
    'B0502': {'B0402': 180, 'B0503': 90, 'B0501': -90},
    'B0503': {'B0403': 180, 'B0504': 90, 'B0502': -90},
    'B0504': {'B0404': 180, 'B0505': 90, 'B0503': -90},
    'B0505': {'B0405': 180, 'B0506': 90, 'B0504': -90},
    'B0506': {'B0406': 180, 'B0505': -90},

    #######################################

    'B0107': {'B0102': 180, 'B0202': 0},
    'B0108': {'B0103': 180, 'B0203': 0},
    'B0109': {'B0104': 180, 'B0204': 0},
    'B0110': {'B0105': 180, 'B0205': 0},
    'B0111': {'B0302': 180, 'B0402': 0},
    'B0112': {'B0303': 180, 'B0403': 0},
    'B0113': {'B0304': 180, 'B0404': 0},
    'B0114': {'B0305': 180, 'B0405': 0}

}