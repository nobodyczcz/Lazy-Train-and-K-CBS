s1x = 1
s1y = 2
s2x = 2
s2y = 1

for i in range(1,9):
    with open("agent/0obs-20x20map-2agents-{}.agents".format(i),"w+") as f:
        f.write("2\n")
        f.write("{},{},{},{},\n".format(s1x,s1y,s2x+i,s2y+i))
        f.write("{},{},{},{},\n".format(s2x,s2y,s1x+i,s1y+i))


