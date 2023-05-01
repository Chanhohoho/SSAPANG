import os

a = int(input())

repeatB = 0
repeatBP = 0
repeatBS = 0
repeatWS = 0
repeatW = 0
repeatWP = 0
repeatBO = 0

def func(Endnum, repeat):
    
    for i in range(1, Endnum):
        if i <= 9:
            i = '0' + str(i)
        
        foldername = botType + '0' + f'{repeat}' + f'{i}'
        
        current_path = os.getcwd()
        os.mkdir(current_path + "/" + foldername)

for _ in range(a):
    botType = input()
    if botType == 'B':
        Endnum = 16
        repeatB += 1
        func(Endnum, repeatB)
    elif botType == 'BP':
        Endnum = 13
        repeatBP += 1
        func(Endnum, repeatBP)
    elif botType == 'BS':
        Endnum = 16
        repeatBS += 1
        func(Endnum, repeatBS)
    elif botType == 'WS':
        Endnum = 9
        repeatWS += 1
        func(Endnum, repeatWS)
    elif botType == 'W':
        Endnum = 9
        repeatW += 1
        func(Endnum, repeatW)
    elif botType == 'WP':
        Endnum = 9
        repeatWP += 1
        func(Endnum, repeatWP)
    elif botType == 'BO':
        Endnum = 16
        repeatBO += 1
        func(Endnum, repeatBO)


