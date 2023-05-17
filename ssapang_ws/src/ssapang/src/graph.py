graph = {

    'BS0101': {'LWI1': 90},
    'BS0102': {'B0102': -90},
    'BS0103': {'LWI2': 90},
    'BS0104': {'B0104': -90},
    'BS0105': {'LWI3': 90},
    'BS0106': {'B0106': -90},
    'BS0107': {'B0107': -90},
    'BS0108': {'WI0': 90},
    'BS0109': {'B0109': -90},
    'BS0110': {'B0110': -90},
    'BS0111': {'RWI3': 90},
    'BS0112': {'B0112': -90},
    'BS0113': {'RWI2': 90},
    'BS0114': {'B0114': -90},
    'BS0115': {'RWI1': 90},

    'LB0100': {'B0101': 0},
    'B0101': {'B0102': 0, 'BS0101': 90},
    'B0102': {'B0103': 0, 'B0202': -90},
    'B0103': {'B0104': 0, 'BS0103': 90},
    'B0104': {'B0105': 0, 'B0204': -90},
    'B0105': {'B0106': 0, 'BS0105': 90},
    'B0106': {'B0107': 0, 'B0206': -90},
    'B0107': {'B0207': -90},
    'B0108': {'BS0108': 90},
    'B0109': {'B0209': -90},
    'B0110': {'B0109': 180, 'B0210': -90},
    'B0111': {'B0110': 180, 'BS0111': 90},
    'B0112': {'B0111': 180, 'B0212': -90},
    'B0113': {'B0112': 180, 'BS0113': 90},
    'B0114': {'B0113': 180, 'B0214': -90},
    'B0115': {'B0114': 180, 'BS0115': 90},
    'RB0116': {'B0115': 180},

    'B0201': {'B0101': 90, 'LB11': 180},
    'B0202': {'BP0101': -90, 'B0201': 180},
    'B0203': {'B0103': 90, 'B0202': 180},
    'B0204': {'BP0103': -90, 'B0203': 180},
    'B0205': {'B0105': 90, 'B0204': 180},
    'B0206': {'BP0105': -90, 'B0205': 180},
    'B0207': {'BP0106': -90, 'B0206': 180},
    'B0208': {'B0108': 90, 'B0207': 180, 'B0209': 0},
    'B0209': {'BP0107': -90, 'B0210': 0},
    'B0210': {'BP0108': -90, 'B0211': 0},
    'B0211': {'B0111': 90, 'B0212': 0},
    'B0212': {'BP0110': -90, 'B0213': 0},
    'B0213': {'B0113': 90, 'B0214': 0},
    'B0214': {'BP0112': -90, 'B0215': 0},
    'B0215': {'B0115': 90, 'RB11': 0},

    'BP0101': {'WBP0101': 0, 'BP0201': -90},
    'BP0102': {'WBP0102': 180, 'B0203': 90, 'BP0103': 0},
    'BP0103': {'WBP0103': 0, 'BP0203': -90},
    'BP0104': {'WBP0104': 180, 'B0205': 90, 'BP0105': 0},
    'BP0105': {'WBP0105': 0, 'BP0205': -90},
    'BP0106': {'WBP0106': 180, 'BP0206': -90},
    'BP0107': {'WBP0107': 0, 'BP0207': -90},
    'BP0108': {'WBP0108': 180, 'BP0208': -90},
    'BP0109': {'WBP0109': 0, 'B0211': 90, 'BP0108': 180},
    'BP0110': {'WBP0110': 180, 'BP0210': -90},
    'BP0111': {'WBP0111': 0, 'B0213': 90, 'BP0110': 180},
    'BP0112': {'WBP0112': 180, 'BP0212': -90},

    'BP0201': {'WBP0201': 0, 'B0302': -90},
    'BP0202': {'WBP0202': 180, 'BP0102': 90},
    'BP0203': {'WBP0203': 0, 'B0304': -90, 'BP0202': 180},
    'BP0204': {'WBP0204': 180, 'BP0104': 90},
    'BP0205': {'WBP0205': 0, 'B0306': -90, 'BP0204': 180},
    'BP0206': {'WBP0206': 180, 'B0307': -90},
    'BP0207': {'WBP0207': 0, 'B0309': -90},
    'BP0208': {'WBP0208': 180, 'B0310': -90, 'BP0209': 0},
    'BP0209': {'WBP0209': 0, 'BP0109': 90},
    'BP0210': {'WBP0210': 180, 'B0312': -90, 'BP0211': 0},
    'BP0211': {'WBP0211': 0, 'BP0111': 90},
    'BP0212': {'WBP0212': 180, 'B0314': -90},

    'LB0300': {'B0301': 0},
    'B0301': {'B0201': 90, 'B0302': 0},
    'B0302': {'B0402': -90, 'B0303': 0},
    'B0303': {'BP0202': 90, 'B0304': 0},
    'B0304': {'B0404': -90, 'B0305': 0},
    'B0305': {'BP0204': 90, 'B0306': 0},
    'B0306': {'B0406': -90, 'B0307': 0},
    'B0307': {'B0407': -90},
    'B0308': {'B0208': 90},
    'B0309': {'B0409': -90},
    'B0310': {'B0410': -90, 'B0309': 180},
    'B0311': {'BP0209': 90, 'B0310': 180},
    'B0312': {'B0412': -90, 'B0311': 180},
    'B0313': {'BP0211': 90, 'B0312': 180},
    'B0314': {'B0414': -90, 'B0313': 180},
    'B0315': {'B0215': 90, 'B0314': 180},
    'RB0316': {'B0315': 180},

    'B0401': {'B0301': 90, 'LB21': 180},
    'B0402': {'BP0301': -90, 'B0401': 180},
    'B0403': {'B0303': 90, 'B0402': 180},
    'B0404': {'BP0303': -90, 'B0403': 180},
    'B0405': {'B0305': 90, 'B0404': 180},
    'B0406': {'BP0305': -90, 'B0405': 180},
    'B0407': {'BP0306': -90, 'B0406': 180},
    'B0408': {'B0308': 90, 'B0407': 180, 'B0409': 0},
    'B0409': {'BP0307': -90, 'B0410': 0},
    'B0410': {'BP0308': -90, 'B0411': 0},
    'B0411': {'B0311': 90, 'B0412': 0},
    'B0412': {'BP0310': -90, 'B0413': 0},
    'B0413': {'B0313': 90, 'B0414': 0},
    'B0414': {'BP0312': -90, 'B0415': 0},
    'B0415': {'B0315': 90, 'RB21': 0},

    'BP0301': {'WBP0301': 0, 'BP0401': -90},
    'BP0302': {'WBP0302': 180, 'B0403': 90, 'BP0303': 0},
    'BP0303': {'WBP0303': 0, 'BP0403': -90},
    'BP0304': {'WBP0304': 180, 'B0405': 90, 'BP0305': 0},
    'BP0305': {'WBP0305': 0, 'BP0405': -90},
    'BP0306': {'WBP0306': 180, 'BP0406': -90},
    'BP0307': {'WBP0307': 0, 'BP0407': -90},
    'BP0308': {'WBP0308': 180, 'BP0408': -90},
    'BP0309': {'WBP0309': 0, 'B0411': 90, 'BP0308': 180},
    'BP0310': {'WBP0310': 180, 'BP0410': -90},
    'BP0311': {'WBP0311': 0, 'B0413': 90, 'BP0310': 180},
    'BP0312': {'WBP0312': 180, 'BP0412': -90},

    'BP0401': {'WBP0401': 0, 'B0502': -90},
    'BP0402': {'WBP0402': 180, 'BP0302': 90},
    'BP0403': {'WBP0403': 0, 'B0504': -90, 'BP0402': 180},
    'BP0404': {'WBP0404': 180, 'BP0304': 90},
    'BP0405': {'WBP0405': 0, 'B0506': -90, 'BP0404': 180},
    'BP0406': {'WBP0406': 180, 'B0507': -90},
    'BP0407': {'WBP0407': 0, 'B0509': -90},
    'BP0408': {'WBP0408': 180, 'B0510': -90, 'BP0409': 0},
    'BP0409': {'WBP0409': 0, 'BP0309': 90},
    'BP0410': {'WBP0410': 180, 'B0512': -90, 'BP0411': 0},
    'BP0411': {'WBP0411': 0, 'BP0311': 90},
    'BP0412': {'WBP0412': 180, 'B0514': -90},

    'LB0500': {'B0501': 0},
    'B0501': {'B0401': 90, 'B0502': 0},
    'B0502': {'B0602': -90, 'B0503': 0},
    'B0503': {'BP0402': 90, 'B0504': 0},
    'B0504': {'B0604': -90, 'B0505': 0},
    'B0505': {'BP0404': 90, 'B0506': 0},
    'B0506': {'B0606': -90, 'B0507': 0},
    'B0507': {'B0607': -90},
    'B0508': {'B0408': 90},
    'B0509': {'B0609': -90},
    'B0510': {'B0610': -90, 'B0509': 180},
    'B0511': {'BP0409': 90, 'B0510': 180},
    'B0512': {'B0612': -90, 'B0511': 180},
    'B0513': {'BP0411': 90, 'B0512': 180},
    'B0514': {'B0614': -90, 'B0513': 180},
    'B0515': {'B0415': 90, 'B0514': 180},
    'RB0516': {'B0515': 180},

    'B0601': {'B0501': 90, 'LB31': 180},
    'B0602': {'BP0501': -90, 'B0601': 180},
    'B0603': {'B0503': 90, 'B0602': 180},
    'B0604': {'BP0503': -90, 'B0603': 180},
    'B0605': {'B0505': 90, 'B0604': 180},
    'B0606': {'BP0505': -90, 'B0605': 180},
    'B0607': {'BP0506': -90, 'B0606': 180},
    'B0608': {'B0508': 90, 'B0607': 180, 'B0609': 0},
    'B0609': {'BP0507': -90, 'B0610': 0},
    'B0610': {'BP0508': -90, 'B0611': 0},
    'B0611': {'B0511': 90, 'B0612': 0},
    'B0612': {'BP0510': -90, 'B0613': 0},
    'B0613': {'B0513': 90, 'B0614': 0},
    'B0614': {'BP0512': -90, 'B0615': 0},
    'B0615': {'B0515': 90, 'RB31': 0},

    'BP0501': {'WBP0501': 0, 'BP0601': -90},
    'BP0502': {'WBP0502': 180, 'B0603': 90, 'BP0503': 0},
    'BP0503': {'WBP0503': 0, 'BP0603': -90},
    'BP0504': {'WBP0504': 180, 'B0605': 90, 'BP0505': 0},
    'BP0505': {'WBP0505': 0, 'BP0605': -90},
    'BP0506': {'WBP0506': 180, 'BP0606': -90},
    'BP0507': {'WBP0507': 0, 'BP0607': -90},
    'BP0508': {'WBP0508': 180, 'BP0608': -90},
    'BP0509': {'WBP0509': 0, 'B0611': 90, 'BP0508': 180},
    'BP0510': {'WBP0510': 180, 'BP0610': -90},
    'BP0511': {'WBP0511': 0, 'B0613': 90, 'BP0510': 180},
    'BP0512': {'WBP0512': 180, 'BP0612': -90},

    'BP0601': {'WBP0601': 0, 'B0702': -90},
    'BP0602': {'WBP0602': 180, 'BP0502': 90},
    'BP0603': {'WBP0603': 0, 'B0704': -90, 'BP0602': 180},
    'BP0604': {'WBP0604': 180, 'BP0504': 90},
    'BP0605': {'WBP0605': 0, 'B0706': -90, 'BP0604': 180},
    'BP0606': {'WBP0606': 180, 'B0707': -90},
    'BP0607': {'WBP0607': 0, 'B0709': -90},
    'BP0608': {'WBP0608': 180, 'B0710': -90, 'BP0609': 0},
    'BP0609': {'WBP0609': 0, 'BP0509': 90},
    'BP0610': {'WBP0610': 180, 'B0712': -90, 'BP0611': 0},
    'BP0611': {'WBP0611': 0, 'BP0511': 90},
    'BP0612': {'WBP0612': 180, 'B0714': -90},

    'LB0700': {'B0701': 0},
    'B0701': {'B0601': 90, 'B0702': 0},
    'B0702': {'B0802': -90, 'B0703': 0},
    'B0703': {'BP0602': 90, 'B0704': 0},
    'B0704': {'B0804': -90, 'B0705': 0},
    'B0705': {'BP0604': 90, 'B0706': 0},
    'B0706': {'B0806': -90, 'B0707': 0},
    'B0707': {'B0807': -90},
    'B0708': {'B0608': 90},
    'B0709': {'B0809': -90},
    'B0710': {'B0810': -90, 'B0709': 180},
    'B0711': {'BP0609': 90, 'B0710': 180},
    'B0712': {'B0812': -90, 'B0711': 180},
    'B0713': {'BP0611': 90, 'B0712': 180},
    'B0714': {'B0814': -90, 'B0713': 180},
    'B0715': {'B0615': 90, 'B0714': 180},
    'RB0716': {'B0715': 180},

    'B0801': {'B0701': 90},
    'B0802': {'B0902': -90, 'B0801': 180},
    'B0803': {'B0703': 90, 'B0802': 180},
    'B0804': {'B0904': -90, 'B0803': 180},
    'B0805': {'B0705': 90, 'B0804': 180},
    'B0806': {'B0906': -90, 'B0805': 180},
    'B0807': {'B0907': -90, 'B0806': 180},
    'B0808': {'B0708': 90, 'B0807': 180, 'B0809': 0},
    'B0809': {'B0909': -90, 'B0810': 0},
    'B0810': {'B0910': -90, 'B0811': 0},
    'B0811': {'B0711': 90, 'B0812': 0},
    'B0812': {'B0912': -90, 'B0813': 0},
    'B0813': {'B0713': 90, 'B0814': 0},
    'B0814': {'B0914': -90, 'B0815': 0},
    'B0815': {'B0715': 90},

    'B0901': {'B0801': 90, 'B0902': 0},
    'B0902': {'BO0102': -90, 'B0903': 0},
    'B0903': {'B0803': 90, 'B0904': 0},
    'B0904': {'BO0104': -90, 'B0905': 0},
    'B0905': {'B0805': 90, 'B0906': 0},
    'B0906': {'BO0106': -90, 'B0907': 0},
    'B0907': {'BO0107': -90},
    'B0908': {'B0808': 90},
    'B0909': {'BO0109': -90},
    'B0910': {'BO0110': -90, 'B0909': 180},
    'B0911': {'B0811': 90, 'B0910': 180},
    'B0912': {'BO0112': -90, 'B0911': 180},
    'B0913': {'B0813': 90, 'B0912': 180},
    'B0914': {'BO0114': -90, 'B0913': 180},
    'B0915': {'B0815': 90, 'B0914': 180},

    'BO0101': {'B0901': 90},
    'BO0102': {'BO0101': 180, },
    'BO0103': {'B0903': 90},
    'BO0104': {'BO0103': 180, },
    'BO0105': {'B0905': 90},
    'BO0106': {'BO0105': 180, },
    'BO0107': {'BO0108': 0, },
    'BO0108': {'B0908': 90},
    'BO0109': {'BO0108': 180},
    'BO0110': {'BO0111': 0},
    'BO0111': {'B0911': 90},
    'BO0112': {'BO0113': 0},
    'BO0113': {'B0913': 90},
    'BO0114': {'BO0115': 0},
    'BO0115': {'B0915': 90},
    ###########################################################
    'RB11': {'RB1111': 0, 'RB12': -90},

    'RB1111': {'RB1112': 0, 'RB1121': -90, },
    'RB1121': {'RB1122': 0, 'RB1131': -90, },
    'RB1131': {'RB1132': 0},

    'RB1112': {'RB1113': 0},
    'RB1122': {'RB1123': 0},
    'RB1132': {'RB1133': 0},

    'RB1113': {'RB0116': 90},
    'RB1123': {'RB1113': 90},
    'RB1133': {'RB1123': 90},

    'RB12': {'RB1211': 0},

    'RB1211': {'RB1212': 0, 'RB1221': -90},
    'RB1221': {'RB1222': 0, 'RB1231': -90},
    'RB1231': {'RB1232': 0},

    'RB1212': {'RB1213': 0},
    'RB1222': {'RB1223': 0},
    'RB1232': {'RB1233': 0},

    'RB1213': {'RB1223': -90},
    'RB1223': {'RB1233': -90},
    'RB1233': {'RB0316': -90},
    ###########################################################
    'RB21': {'RB2111': 0, 'RB22': -90},

    'RB2111': {'RB2112': 0, 'RB2121': -90},
    'RB2121': {'RB2122': 0, 'RB2131': -90},
    'RB2131': {'RB2132': 0, },

    'RB2112': {'RB2113': 0},
    'RB2122': {'RB2123': 0},
    'RB2132': {'RB2133': 0},

    'RB2113': {'RB0316': 90},
    'RB2123': {'RB2113': 90},
    'RB2133': {'RB2123': 90},

    'RB22': {'RB2211': 0},

    'RB2211': {'RB2212': 0, 'RB2221': -90},
    'RB2221': {'RB2222': 0, 'RB2231': -90},
    'RB2231': {'RB2232': 0},

    'RB2212': {'RB2213': 0},
    'RB2222': {'RB2223': 0},
    'RB2232': {'RB2233': 0},

    'RB2213': {'RB2223': -90},
    'RB2223': {'RB2233': -90},
    'RB2233': {'RB0516': -90},
    ###########################################################
    'RB31': {'RB3111': 0, 'RB32': -90},

    'RB3111': {'RB3112': 0, 'RB3121': -90},
    'RB3121': {'RB3122': 0, 'RB3131': -90},
    'RB3131': {'RB3132': 0},

    'RB3112': {'RB3113': 0},
    'RB3122': {'RB3123': 0},
    'RB3132': {'RB3133': 0},

    'RB3113': {'RB0516': 90},
    'RB3123': {'RB3113': 90},
    'RB3133': {'RB3123': 90},

    'RB32': {'RB3211': 0},

    'RB3211': {'RB3212': 0, 'RB3221': -90},
    'RB3221': {'RB3222': 0, 'RB3231': -90},
    'RB3231': {'RB3232': 0},

    'RB3212': {'RB3213': 0},
    'RB3222': {'RB3223': 0},
    'RB3232': {'RB3233': 0},

    'RB3213': {'RB3223': -90},
    'RB3223': {'RB3233': -90},
    'RB3233': {'RB0716': -90},
    ###########################################################
    'LB11': {'LB1111': 180, 'LB12': -90},

    'LB1111': {'LB1112': 180, 'LB1121': -90},
    'LB1121': {'LB1122': 180, 'LB1131': -90},
    'LB1131': {'LB1132': 180},

    'LB1112': {'LB1113': 180},
    'LB1122': {'LB1123': 180},
    'LB1132': {'LB1133': 180},

    'LB1113': {'LB0100': 90},
    'LB1123': {'LB1113': 90},
    'LB1133': {'LB1123': 90},

    'LB12': {'LB1211': 180},

    'LB1211': {'LB1212': 180, 'LB1221': -90},
    'LB1221': {'LB1222': 180, 'LB1231': -90},
    'LB1231': {'LB1232': 180},

    'LB1212': {'LB1213': 180},
    'LB1222': {'LB1223': 180},
    'LB1232': {'LB1233': 180},

    'LB1213': {'LB1223': -90},
    'LB1223': {'LB1233': -90},
    'LB1233': {'LB0300': -90},
    ###########################################################
    'LB21': {'LB2111': 180, 'LB22': -90},

    'LB2111': {'LB2112': 180, 'LB2121': -90},
    'LB2121': {'LB2122': 180, 'LB2131': -90},
    'LB2131': {'LB2132': 180},

    'LB2112': {'LB2113': 180},
    'LB2122': {'LB2123': 180},
    'LB2132': {'LB2133': 180},

    'LB2113': {'LB0300': 90},
    'LB2123': {'LB2113': 90},
    'LB2133': {'LB2123': 90},

    'LB22': {'LB2211': 180},

    'LB2211': {'LB2212': 180, 'LB2221': -90},
    'LB2221': {'LB2222': 180, 'LB2231': -90},
    'LB2231': {'LB2232': 180},

    'LB2212': {'LB2213': 180},
    'LB2222': {'LB2223': 180},
    'LB2232': {'LB2233': 180},

    'LB2213': {'LB2223': -90},
    'LB2223': {'LB2233': -90},
    'LB2233': {'LB0500': -90},
    ###########################################################
    'LB31': {'LB3111': 180, 'LB32': -90},

    'LB3111': {'LB3112': 180, 'LB3121': -90},
    'LB3121': {'LB3122': 180, 'LB3131': -90},
    'LB3131': {'LB3132': 180},

    'LB3112': {'LB3113': 180},
    'LB3122': {'LB3123': 180},
    'LB3132': {'LB3133': 180},

    'LB3113': {'LB0500': 90},
    'LB3123': {'LB3113': 90},
    'LB3133': {'LB3123': 90},

    'LB32': {'LB3211': 180},

    'LB3211': {'LB3212': 180, 'LB3221': -90},
    'LB3221': {'LB3222': 180, 'LB3231': -90},
    'LB3231': {'LB3232': 180},

    'LB3212': {'LB3213': 180},
    'LB3222': {'LB3223': 180},
    'LB3232': {'LB3233': 180},

    'LB3213': {'LB3223': -90},
    'LB3223': {'LB3233': -90},
    'LB3233': {'LB0700': -90},
    ###########################################################
    'LW11': {'LW12': 180, 'LW21': -90},
    'LW12': {'LW13': 180, 'LW22': -90},
    'LW13': {'LW14': 180, 'LW23': -90},
    'LW14': {'LW15': 180, 'LW24': -90},
    'LW15': {'LW16': 180, 'LW25': -90},
    'LW16': {'LW26': -90},
    'LW21': {'LW31': -90},
    'LW22': {'LW32': -90},
    'LW23': {'LW33': -90},
    'LW24': {'LW34': -90},
    'LW25': {'LW35': -90},
    'LW26': {'LW36': -90},
    'LW31': {'LW32': 180},
    'LW32': {'LW33': 180},
    'LW33': {'LW34': 180},
    'LW34': {'LW35': 180},
    'LW35': {'LW36': 180},
    'LW36': {'LWO31': -90},

    'LWO11': {'LWO12': -90},
    'LWO12': {'LWO13': -90, 'LWI2': 0},
    'LWO13': {'LWO14': -90},
    'LWO14': {'BS0102': -90},
    'LWO21': {'LWO22': -90, 'LWO11': 180},
    'LWO22': {'LWO23': -90, 'LWI3': 0},
    'LWO23': {'LWO24': -90},
    'LWO24': {'BS0104': -90},
    'LWO31': {'LWO32': -90, 'LWO21': 180, 'LWO41': 0},
    'LWO32': {'LWO33': -90, 'LWO42': 0},
    'LWO33': {'LWO34': -90},
    'LWO34': {'BS0106': -90},
    'LWO41': {'LWO42': -90},
    'LWO42': {'LWO43': -90, 'WI0': 0},
    'LWO43': {'LWO44': -90},
    'LWO44': {'BS0107': -90},

    'RW11': {'RW12': 0, 'RW21': -90},
    'RW12': {'RW13': 0, 'RW22': -90},
    'RW13': {'RW14': 0, 'RW23': -90},
    'RW14': {'RW15': 0, 'RW24': -90},
    'RW15': {'RW16': 0, 'RW25': -90},
    'RW16': {'RW26': -90},
    'RW21': {'RW31': -90},
    'RW22': {'RW32': -90},
    'RW23': {'RW33': -90},
    'RW24': {'RW34': -90},
    'RW25': {'RW35': -90},
    'RW26': {'RW36': -90},
    'RW31': {'RW32': 0, },
    'RW32': {'RW33': 0, },
    'RW33': {'RW34': 0, },
    'RW34': {'RW35': 0, },
    'RW35': {'RW36': 0, },
    'RW36': {'RWO31': -90},

    'RWO11': {'RWO12': -90},
    'RWO12': {'RWO13': -90, 'RWI2': 180},
    'RWO13': {'RWO14': -90},
    'RWO14': {'BS0114': -90},
    'RWO21': {'RWO22': -90, 'RWO11': 0},
    'RWO22': {'RWO23': -90, 'RWI3': 0},
    'RWO23': {'RWO24': -90},
    'RWO24': {'BS0112': -90},
    'RWO31': {'RWO32': -90, 'RWO21': 0, 'RWO41': 180},
    'RWO32': {'RWO33': -90, 'RWO42': 180},
    'RWO33': {'RWO34': -90},
    'RWO34': {'BS0110': -90},
    'RWO41': {'RWO42': -90},
    'RWO42': {'RWO43': -90, 'WI0': 180},
    'RWO43': {'RWO44': -90},
    'RWO44': {'BS0109': -90},

    'LWI1': {'LWO12': 0},
    'LWI2': {'LWO22': 0},
    'LWI3': {'LWO32': 0},

    'WI0': {'WI1': 90},
    'WI1': {'WI2': 90},
    'WI2': {'LW11': 180, 'RW11': 0},

    'RWI1': {'RWO12': 180},
    'RWI2': {'RWO22': 180},
    'RWI3': {'RWO32': 180},
    ###########################################################
    'WBP0101': {'BP0101': 180},
    'WBP0102': {'BP0102': 0},
    'WBP0103': {'BP0103': 180},
    'WBP0104': {'BP0104': 0},
    'WBP0105': {'BP0105': 180},
    'WBP0106': {'BP0106': 0},
    'WBP0107': {'BP0107': 180},
    'WBP0108': {'BP0108': 0},
    'WBP0109': {'BP0109': 180},
    'WBP0110': {'BP0110': 0},
    'WBP0111': {'BP0111': 180},
    'WBP0112': {'BP0112': 0},

    'WBP0201': {'BP0201': 180},
    'WBP0202': {'BP0202': 0},
    'WBP0203': {'BP0203': 180},
    'WBP0204': {'BP0204': 0},
    'WBP0205': {'BP0205': 180},
    'WBP0206': {'BP0206': 0},
    'WBP0207': {'BP0207': 180},
    'WBP0208': {'BP0208': 0},
    'WBP0209': {'BP0209': 180},
    'WBP0210': {'BP0210': 0},
    'WBP0211': {'BP0211': 180},
    'WBP0212': {'BP0212': 0},

    'WBP0301': {'BP0301': 180},
    'WBP0302': {'BP0302': 0},
    'WBP0303': {'BP0303': 180},
    'WBP0304': {'BP0304': 0},
    'WBP0305': {'BP0305': 180},
    'WBP0306': {'BP0306': 0},
    'WBP0307': {'BP0307': 180},
    'WBP0308': {'BP0308': 0},
    'WBP0309': {'BP0309': 180},
    'WBP0310': {'BP0310': 0},
    'WBP0311': {'BP0311': 180},
    'WBP0312': {'BP0312': 0},

    'WBP0401': {'BP0401': 180},
    'WBP0402': {'BP0402': 0},
    'WBP0403': {'BP0403': 180},
    'WBP0404': {'BP0404': 0},
    'WBP0405': {'BP0405': 180},
    'WBP0406': {'BP0406': 0},
    'WBP0407': {'BP0407': 180},
    'WBP0408': {'BP0408': 0},
    'WBP0409': {'BP0409': 180},
    'WBP0410': {'BP0410': 0},
    'WBP0411': {'BP0411': 180},
    'WBP0412': {'BP0412': 0},

    'WBP0501': {'BP0501': 180},
    'WBP0502': {'BP0502': 0},
    'WBP0503': {'BP0503': 180},
    'WBP0504': {'BP0504': 0},
    'WBP0505': {'BP0505': 180},
    'WBP0506': {'BP0506': 0},
    'WBP0507': {'BP0507': 180},
    'WBP0508': {'BP0508': 0},
    'WBP0509': {'BP0509': 180},
    'WBP0510': {'BP0510': 0},
    'WBP0511': {'BP0511': 180},
    'WBP0512': {'BP0512': 0},

    'WBP0601': {'BP0601': 180},
    'WBP0602': {'BP0602': 0},
    'WBP0603': {'BP0603': 180},
    'WBP0604': {'BP0604': 0},
    'WBP0605': {'BP0605': 180},
    'WBP0606': {'BP0606': 0},
    'WBP0607': {'BP0607': 180},
    'WBP0608': {'BP0608': 0},
    'WBP0609': {'BP0609': 180},
    'WBP0610': {'BP0610': 0},
    'WBP0611': {'BP0611': 180},
    'WBP0612': {'BP0612': 0},

}