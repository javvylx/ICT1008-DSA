



def astar(start,map,table):
    for i in range(len(table)):
        if(table[i][1]=='c'):
            table[i][1] = 'v'
        if (start==table[i][0]):
            table[i][1]='c'
            beg=i
            table[i][4] = hc[i]


    nbr = []
    for i in range(len(map)):
        if ((map[beg][i]) > 0):
            if(table[i][1]!='v'):
                if (((table[i][2]) < (table[beg][2] + map[beg][i])) and (table[i][2] > 0)):
                    continue
                else:
                    table[i][2] = table[beg][2]+map[beg][i]
                    table[i][4] = table[i][2] + table[i][3]
                    table[i][5] = node[beg]
                nbr.append(i)



    td = table[nbr[0]][4]
    next = table[nbr[0]][0]
    for i in range(1, len(nbr)):
        if ((table[nbr[i]][4]) < td):
            td = table[nbr[i]][4]
            next = table[nbr[i]][0]


    return next



# get user input
node= ['a','b','c','d','e','f','z']
hc=[14,12,11,6,4,11,0]
#hc=[21,14,18,18,5,8,0]

start='a'
target='z'
path=[]
path.append(start)


table=[[0,0,0,0,0,0],
       [0,0,0,0,0,0],
       [0,0,0,0,0,0],
       [0,0,0,0,0,0],
       [0,0,0,0,0,0],
       [0,0,0,0,0,0],
       [0,0,0,0,0,0]]

map=[[0,4,3,0,0,0,0],
    [4,0,0,0,12,5,0],
    [3,0,0,7,10,0,0],
    [0,0,7,0,2,0,0],
    [0,12,10,2,0,0,5],
    [0,5,0,0,0,0,16],
    [0,0,0,0,5,16,0]]

"""map=[[0,9,4,7,0,0,0],
    [9,0,0,0,11,0,0],
    [4,0,0,0,17,12,0],
    [7,0,0,0,0,14,0],
    [0,11,17,0,0,0,5],
    [0,0,12,14,0,0,9],
    [0,0,0,0,5,9,0]]"""


for i in range(len(node)):
    table[i][0]=node[i]

for i in range(len(node)):
    table[i][3]=hc[i]

pointer=astar(start,map,table)
path.append(pointer)
while(pointer!=target):
    pointer=astar(pointer,map,table)
    path.append(pointer)







print(" N, S, D, H, T, P")
for i in range(7):
    if(table[i][0]==target):
        distance=table[i][4]
    print(table[i])
print(distance)
print(path)

