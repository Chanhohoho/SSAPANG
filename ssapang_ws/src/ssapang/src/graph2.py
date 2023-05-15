graph = {

    'BS0101': {'LWI1':90},
    'BS0102': {'B0102': -90},
    'BS0103': {'LWI2':90},
    'BS0104': {'B0104': -90},
    'BS0105': {'LWI3':90},
    'BS0106': {'B0106': -90},
    'BS0107': {'B0107': -90},
    'BS0108': {'WI0':90},
    'BS0109': {'B0109': -90},
    'BS0110': {'B0110': -90},
    'BS0111': {'RWI3':90},
    'BS0112': {'B0112': -90},
    'BS0113': {'RWI2':90},
    'BS0114': {'B0114': -90},
    'BS0115': {'RWI1':90},

    'B0100': {'B0101':0},
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
    'B0116': {'B0115': 180},

    'B0201': {'B0101': 90, 'LB11':180},
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
    'B0215': {'B0115': 90, 'RB11':0},

    'BP0101': {'BP0201': -90},
    'BP0102': {'B0203': 90, 'BP0103': 0},
    'BP0103': {'BP0203': -90},
    'BP0104': {'B0205': 90, 'BP0105': 0},
    'BP0105': {'BP0205': -90},
    'BP0106': {'BP0206': -90},
    'BP0107': {'BP0207': -90},
    'BP0108': {'BP0208': -90},
    'BP0109': {'B0211': 90, 'BP0108': 180},
    'BP0110': {'BP0210': -90},
    'BP0111': {'B0213': 90, 'BP0110': 180},
    'BP0112': {'BP0212': -90},

    'BP0201': {'B0302': -90},
    'BP0202': {'BP0102': 90},
    'BP0203': {'B0304': -90, 'BP0202': 180},
    'BP0204': {'BP0104': 90},
    'BP0205': {'B0306': -90, 'BP0204': 180},
    'BP0206': {'B0307': -90},
    'BP0207': {'B0309': -90},
    'BP0208': {'B0310': -90, 'BP0209': 0},
    'BP0209': {'BP0109': 90},
    'BP0210': {'B0312': -90, 'BP0211': 0},
    'BP0211': {'BP0111': 90},
    'BP0212': {'B0314': -90},

    'B0300': {'B0301': 0},
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
    'B0316': {'B0315': 180},

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
    'B0415': {'B0315': 90, 'RB21':0},

    'BP0301': {'BP0401': -90},
    'BP0302': {'B0403': 90, 'BP0303': 0},
    'BP0303': {'BP0403': -90},
    'BP0304': {'B0405': 90, 'BP0305': 0},
    'BP0305': {'BP0405': -90},
    'BP0306': {'BP0406': -90},
    'BP0307': {'BP0407': -90},
    'BP0308': {'BP0408': -90},
    'BP0309': {'B0411': 90, 'BP0308': 180},
    'BP0310': {'BP0410': -90},
    'BP0311': {'B0413': 90, 'BP0310': 180},
    'BP0312': {'BP0412': -90},

    'BP0401': {'B0502': -90},
    'BP0402': {'BP0302': 90},
    'BP0403': {'B0504': -90, 'BP0402': 180},
    'BP0404': {'BP0304': 90},
    'BP0405': {'B0506': -90, 'BP0404': 180},
    'BP0406': {'B0507': -90},
    'BP0407': {'B0509': -90},
    'BP0408': {'B0510': -90, 'BP0409': 0},
    'BP0409': {'BP0309': 90},
    'BP0410': {'B0512': -90, 'BP0411': 0},
    'BP0411': {'BP0311': 90},
    'BP0412': {'B0514': -90},

    'B0500': {'B0501': 0},
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
    'B0516': {'B0515':180},

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
    'B0615': {'B0515': 90, 'RB31':0},

    'BP0501': {'BP0601': -90},
    'BP0502': {'B0603': 90, 'BP0503': 0},
    'BP0503': {'BP0603': -90},
    'BP0504': {'B0605': 90, 'BP0505': 0},
    'BP0505': {'BP0605': -90},
    'BP0506': {'BP0606': -90},
    'BP0507': {'BP0607': -90},
    'BP0508': {'BP0608': -90},
    'BP0509': {'B0611': 90, 'BP0508': 180},
    'BP0510': {'BP0610': -90},
    'BP0511': {'B0613': 90, 'BP0510': 180},
    'BP0512': {'BP0612': -90},

    'BP0601': {'B0702': -90},
    'BP0602': {'BP0502': 90},
    'BP0603': {'B0704': -90, 'BP0602': 180},
    'BP0604': {'BP0504': 90},
    'BP0605': {'B0706': -90, 'BP0604': 180},
    'BP0606': {'B0707': -90},
    'BP0607': {'B0709': -90},
    'BP0608': {'B0710': -90, 'BP0609': 0},
    'BP0609': {'BP0509': 90},
    'BP0610': {'B0712': -90, 'BP0611': 0},
    'BP0611': {'BP0511': 90},
    'BP0612': {'B0714': -90},

    'B0700': {'B0701':0},
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
    'B0716': {'B0715': 180},

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
    'BO0102': {'BO0101': 180,},
    'BO0103': {'B0903': 90},
    'BO0104': {'BO0103': 180,},
    'BO0105': {'B0905': 90},
    'BO0106': {'BO0105': 180,},
    'BO0107': {'BO0108': 0,},
    'BO0108': {'B0908': 90},
    'BO0109': {'BO0108': 180},
    'BO0110': {'BO0111': 0},
    'BO0111': {'B0911': 90},
    'BO0112': {'BO0113': 0},
    'BO0113': {'B0913': 90},
    'BO0114': {'BO0115': 0},
    'BO0115': {'B0915': 90},

    'RB11':{'RB1111':0, 'RB12':-90},

    'RB1111':{'RB1112':0, 'RB1121': -90,},
    'RB1121':{'RB1122':0, 'RB1131': -90,},
    'RB1131':{'RB1132':0, 'RB1141': -90,},
    'RB1141':{'RB1142':0, 'RB1151': -90,},
    'RB1151':{'RB1152':0},

    'RB1112':{'RB1113':0},
    'RB1122':{'RB1123':0},
    'RB1132':{'RB1133':0},
    'RB1142':{'RB1143':0},
    'RB1152':{'RB1153':0},

    'RB1113':{'B0116': 90},
    'RB1123':{'RB1113':90},
    'RB1133':{'RB1123':90},
    'RB1143':{'RB1133':90},
    'RB1153':{'RB1143':90},
    
    'RB12':{'RB1211':0},

    'RB1211':{'RB1212':0,'RB1221':-90},
    'RB1221':{'RB1222':0,'RB1231':-90},
    'RB1231':{'RB1232':0,'RB1241':-90},
    'RB1241':{'RB1242':0,'RB1251':-90},
    'RB1251':{'RB1252':0,},

    'RB1212':{'RB1213':0},
    'RB1222':{'RB1223':0},
    'RB1232':{'RB1233':0},
    'RB1242':{'RB1243':0},
    'RB1252':{'RB1253':0},

    'RB1213':{'RB1223':-90},
    'RB1223':{'RB1233':-90},
    'RB1233':{'RB1243':-90},
    'RB1243':{'RB1253':-90},
    'RB1253':{'B0316' :-90},

    'RB21':{'RB2111': 0, 'RB22': -90},

    'RB2111':{'RB2112':0,'RB2121':-90},
    'RB2121':{'RB2122':0,'RB2131':-90},
    'RB2131':{'RB2132':0,'RB2141':-90},
    'RB2141':{'RB2142':0,'RB2151':-90},
    'RB2151':{'RB2152':0,},

    'RB2112':{'RB2113':0},
    'RB2122':{'RB2123':0},
    'RB2132':{'RB2133':0},
    'RB2142':{'RB2143':0},
    'RB2152':{'RB2153':0},

    'RB2113':{'B0316': 90},
    'RB2123':{'RB2113':90},
    'RB2133':{'RB2123':90},
    'RB2143':{'RB2133':90},
    'RB2153':{'RB2143':90},

    'RB22':{'RB2211': 0},

    'RB2211':{'RB2212':0,'RB2221':-90},
    'RB2221':{'RB2222':0,'RB2231':-90},
    'RB2231':{'RB2232':0,'RB2241':-90},
    'RB2241':{'RB2242':0,'RB2251':-90},
    'RB2251':{'RB2252':0,},

    'RB2212':{'RB2213':0,},
    'RB2222':{'RB2223':0,},
    'RB2232':{'RB2233':0,},
    'RB2242':{'RB2243':0,},
    'RB2252':{'RB2253':0,},

    'RB2213':{'RB2223':-90},
    'RB2223':{'RB2233':-90},
    'RB2233':{'RB2243':-90},
    'RB2243':{'RB2253':-90},
    'RB2253':{'B0516' :-90},

    'RB31':{'RB3111': 0, 'RB32': -90},

    'RB3111':{'RB3112':0, 'RB3121':-90},
    'RB3121':{'RB3122':0, 'RB3131':-90},
    'RB3131':{'RB3132':0, 'RB3141':-90},
    'RB3141':{'RB3142':0, 'RB3151':-90},
    'RB3151':{'RB3152':0, },

    'RB3112':{'RB3113':0},
    'RB3122':{'RB3123':0},
    'RB3132':{'RB3133':0},
    'RB3142':{'RB3143':0},
    'RB3152':{'RB3153':0},

    'RB3113':{'B0516': 90},
    'RB3123':{'RB3113':90},
    'RB3133':{'RB3123':90},
    'RB3143':{'RB3133':90},
    'RB3153':{'RB3143':90},

    'RB32':{'RB3211': 0},

    'RB3211':{'RB3212':0, 'RB3221':-90},
    'RB3221':{'RB3222':0, 'RB3231':-90},
    'RB3231':{'RB3232':0, 'RB3241':-90},
    'RB3241':{'RB3242':0, 'RB3251':-90},
    'RB3251':{'RB3252':0, },

    'RB3212':{'RB3213':0},
    'RB3222':{'RB3223':0},
    'RB3232':{'RB3233':0},
    'RB3242':{'RB3243':0},
    'RB3252':{'RB3253':0},

    'RB3213':{'RB3223':-90},
    'RB3223':{'RB3233':-90},
    'RB3233':{'RB3243':-90},
    'RB3243':{'RB3253':-90},
    'RB3253':{'B0716': -90},

    'LB11':{'LB1111': 180, 'LB12':-90},

    'LB1111':{'LB1112':180,'LB1121': -90},
    'LB1121':{'LB1122':180,'LB1131': -90},
    'LB1131':{'LB1132':180,'LB1141': -90},
    'LB1141':{'LB1142':180,'LB1151': -90},
    'LB1151':{'LB1152':180, },

    'LB1112':{'LB1113':180},
    'LB1122':{'LB1123':180},
    'LB1132':{'LB1133':180},
    'LB1142':{'LB1143':180},
    'LB1152':{'LB1153':180},

    'LB1113':{'B0100': 90},
    'LB1123':{'LB1113':90},
    'LB1133':{'LB1123':90},
    'LB1143':{'LB1133':90},
    'LB1153':{'LB1143':90},

    'LB12':{'LB1211':180},

    'LB1211':{'LB1212':180, 'LB1221':-90},
    'LB1221':{'LB1222':180, 'LB1231':-90},
    'LB1231':{'LB1232':180, 'LB1241':-90},
    'LB1241':{'LB1242':180, 'LB1251':-90},
    'LB1251':{'LB1252':180, },

    'LB1212':{'LB1213':180},
    'LB1222':{'LB1223':180},
    'LB1232':{'LB1233':180},
    'LB1242':{'LB1243':180},
    'LB1252':{'LB1253':180},

    'LB1213':{'LB1223':-90},
    'LB1223':{'LB1233':-90},
    'LB1233':{'LB1243':-90},
    'LB1243':{'LB1253':-90},
    'LB1253':{'B0300': -90},

    'LB21':{'LB2111':180, 'LB22': -90},

    'LB2111':{'LB2112':180,'LB2121':-90},
    'LB2121':{'LB2122':180,'LB2131':-90},
    'LB2131':{'LB2132':180,'LB2141':-90},
    'LB2141':{'LB2142':180,'LB2151':-90},
    'LB2151':{'LB2152':180,},

    'LB2112':{'LB2113':180},
    'LB2122':{'LB2123':180},
    'LB2132':{'LB2133':180},
    'LB2142':{'LB2143':180},
    'LB2152':{'LB2153':180},

    'LB2113':{'B0300': 90},
    'LB2123':{'LB2113':90},
    'LB2133':{'LB2123':90},
    'LB2143':{'LB2133':90},
    'LB2153':{'LB2143':90},

    'LB22':{'LB2211': 180},

    'LB2211':{'LB2212':180,'LB2221':-90},
    'LB2221':{'LB2222':180,'LB2231':-90},
    'LB2231':{'LB2232':180,'LB2241':-90},
    'LB2241':{'LB2242':180,'LB2251':-90},
    'LB2251':{'LB2252':180,},

    'LB2212':{'LB2213':180,},
    'LB2222':{'LB2223':180,},
    'LB2232':{'LB2233':180,},
    'LB2242':{'LB2243':180,},
    'LB2252':{'LB2253':180,},

    'LB2213':{'LB2223':-90},
    'LB2223':{'LB2233':-90},
    'LB2233':{'LB2243':-90},
    'LB2243':{'LB2253':-90},
    'LB2253':{'B0500': -90},

    'LB31':{'LB3111': 180, 'LB32': -90},

    'LB3111':{'LB3112':180, 'LB3121':-90},
    'LB3121':{'LB3122':180, 'LB3131':-90},
    'LB3131':{'LB3132':180, 'LB3141':-90},
    'LB3141':{'LB3142':180, 'LB3151':-90},
    'LB3151':{'LB3152':180, },

    'LB3112':{'LB3113':180},
    'LB3122':{'LB3123':180},
    'LB3132':{'LB3133':180},
    'LB3142':{'LB3143':180},
    'LB3152':{'LB3153':180},

    'LB3113':{'B0500': 90},
    'LB3123':{'LB3113':90},
    'LB3133':{'LB3123':90},
    'LB3143':{'LB3133':90},
    'LB3153':{'LB3143':90},

    'LB32':{'LB3211': 180},

    'LB3211':{'LB3212':180, 'LB3221':-90},
    'LB3221':{'LB3222':180, 'LB3231':-90},
    'LB3231':{'LB3232':180, 'LB3241':-90},
    'LB3241':{'LB3242':180, 'LB3251':-90},
    'LB3251':{'LB3252':180,},

    'LB3212':{'LB3213':180},
    'LB3222':{'LB3223':180},
    'LB3232':{'LB3233':180},
    'LB3242':{'LB3243':180},
    'LB3252':{'LB3253':180},

    'LB3213':{'LB3223':-90},
    'LB3223':{'LB3233':-90},
    'LB3233':{'LB3243':-90},
    'LB3243':{'LB3253':-90},
    'LB3253':{'B0700': -90},


    'LW11':{'LW12':180,'LW21':-90},
    'LW12':{'LW13':180,'LW22':-90},
    'LW13':{'LW14':180,'LW23':-90},
    'LW14':{'LW15':180,'LW24':-90},
    'LW15':{'LW16':180,'LW25':-90},
    'LW16':{'LW26':-90},
    'LW21':{'LW22':180,'LW31':-90},
    'LW22':{'LW23':180,'LW32':-90},
    'LW23':{'LW24':180,'LW33':-90},
    'LW24':{'LW25':180,'LW34':-90},
    'LW25':{'LW26':180,'LW35':-90},
    'LW26':{'LW36':-90},
    'LW31':{'LW32':180},
    'LW32':{'LW33':180},
    'LW33':{'LW34':180},
    'LW34':{'LW35':180},
    'LW35':{'LW36':180},
    'LW36':{'LWO31':-90},

    'LWO11':{'LWO12':-90},
    'LWO12':{'LWO13':-90, 'LWI2':0},
    'LWO13':{'LWO14':-90},
    'LWO14':{'BS0102':-90},
    'LWO21':{'LWO22':-90, 'LWO11':180},
    'LWO22':{'LWO23':-90, 'LWI3':0},
    'LWO23':{'LWO24':-90},
    'LWO24':{'BS0104':-90},
    'LWO31':{'LWO32':-90, 'LWO21':180, 'LWO41':0},
    'LWO32':{'LWO33':-90, 'LWO42':0},
    'LWO33':{'LWO34':-90},
    'LWO34':{'BS0106':-90},
    'LWO41':{'LWO42':-90},
    'LWO42':{'LWO43':-90, 'WI0':0},
    'LWO43':{'LWO44':-90},
    'LWO44':{'BS0107':-90},

    'RW11':{'RW12':0, 'RW21':-90},
    'RW12':{'RW13':0, 'RW22':-90},
    'RW13':{'RW14':0, 'RW23':-90},
    'RW14':{'RW15':0, 'RW24':-90},
    'RW15':{'RW16':0, 'RW25':-90},
    'RW16':{'RW26':-90},
    'RW21':{'RW22':0, 'RW31':-90},
    'RW22':{'RW23':0, 'RW32':-90},
    'RW23':{'RW24':0, 'RW33':-90},
    'RW24':{'RW25':0, 'RW34':-90},
    'RW25':{'RW26':0, 'RW35':-90},
    'RW26':{'RW36':-90},
    'RW31':{'RW32':0,},
    'RW32':{'RW33':0,},
    'RW33':{'RW34':0,},
    'RW34':{'RW35':0,},
    'RW35':{'RW36':0,},
    'RW36':{'RWO31':-90},

    'RWO11':{'RWO12':-90},
    'RWO12':{'RWO13':-90, 'RWI2':180},
    'RWO13':{'RWO14':-90},
    'RWO14':{'BS0114':-90},
    'RWO21':{'RWO22':-90, 'RWO11':0},
    'RWO22':{'RWO23':-90, 'RWI3':0},
    'RWO23':{'RWO24':-90},
    'RWO24':{'BS0112':-90},
    'RWO31':{'RWO32':-90, 'RWO21':0, 'RWO41':180},
    'RWO32':{'RWO33':-90, 'RWO42':180},
    'RWO33':{'RWO34':-90},
    'RWO34':{'BS0110':-90},
    'RWO41':{'RWO42':-90},
    'RWO42':{'RWO43':-90, 'WI0':180},
    'RWO43':{'RWO44':-90},
    'RWO44':{'BS0109':-90},

    'LWI1':{'LWO12':0},
    'LWI2':{'LWO22':0},
    'LWI3':{'LWO32':0},

    'WI0':{'WI1':90},
    'WI1':{'WI2':90},
    'WI2':{'LW11':180, 'RW11':0},

    'RWI1':{'RWO12':180},
    'RWI2':{'RWO22':180},
    'RWI3':{'RWO32':180}

}