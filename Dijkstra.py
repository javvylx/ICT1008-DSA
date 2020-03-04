



def dijkstra(start,map,table):
    for i in range(len(table)):
        if(table[i][1]=='c'):
            table[i][1] = 'v'
        if (start==table[i][0]):
            table[i][1]='c'
            beg=i


    nbr = []
    cont=0
    for i in range(len(map)):
        if ((map[beg][i]) > 0):
            if(table[i][1]!='v'):
                if(((table[i][2])<(table[beg][2]+map[beg][i])) and (table[i][2]>0)):
                    table[i][2] = table[i][2]
                    table[i][3] = table[i][3]
                    nbr.append(i)
                else:
                    table[i][2] = table[beg][2] + map[beg][i]
                    table[i][3] = node[beg]
                    nbr.append(i)
                    table[i][4].append(start)


    td = table[nbr[0]][2]
    next = table[nbr[0]][0]
    for i in range(1, len(nbr)):
        if ((table[nbr[i]][2]) < td):
            td = table[nbr[i]][2]
            next = table[nbr[i]][0]


    return next



# get user input
node= ['A','B','C','D','E','Z']


start='A'
target='Z'



"""table=[[0,0,0,0,[]],
       [0,0,0,0,[]],
       [0,0,0,0,[]],
       [0,0,0,0,[]],
       [0,0,0,0,[]],
       [0,0,0,0,[]],
       [0,0,0,0,[]]]"""

table=[[0,0,0,0,[]],
       [0,0,0,0,[]],
       [0,0,0,0,[]],
       [0,0,0,0,[]],
       [0,0,0,0,[]],
       [0,0,0,0,[]]]

map=[[0,4,2,0,0,0],
    [4,0,1,5,0,0],
    [2,1,0,8,10,0],
    [0,5,8,0,2,6],
    [0,0,10,2,0,5],
    [0,0,0,6,5,0]]

"""map=[[0,4,3,0,0,0,0],
    [4,0,0,0,12,5,0],
    [3,0,0,7,10,0,0],
    [0,0,7,0,2,0,0],
    [0,12,10,2,0,0,5],
    [0,5,0,0,0,0,16],
    [0,0,0,0,5,16,0]]"""


for i in range(len(node)):
    table[i][0]=node[i]




pointer=dijkstra(start,map,table)
while(pointer!=target):
    pointer=dijkstra(pointer,map,table)








print(" N, S, D, P")
for i in range(len(table)):
    if(table[i][0]==target):
        distance=table[i][2]
        point = table[i][3][0]
    print(table[i])
print(distance)
path=[]
path.append(target)
path.append(point)
while(point!=start):
    for i in range(len(table)):
        if(table[i][0]==point):
            distance=table[i][2]
            point = table[i][3][0]
            path.append(table[i][3])
print(path)