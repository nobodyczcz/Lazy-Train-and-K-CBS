import subprocess, random,os

theMap = os.path.abspath('C:/Users/czcz2/Google Drive/MIT/GCITR/CBSH-rect/map/0obs-20x20')
outputFolder = os.path.abspath('C:/Users/czcz2/Google Drive/MIT/GCITR/CBSH-rect/outputs/0obs-20x20-random')
agentsFolder = os.path.abspath('C:/Users/czcz2/Google Drive/MIT/GCITR/CBSH-rect/agent/0obs-20x20-random')
algos = ["CBS","CBSH","CBSH-CR","CBSH-R"]


try:
    os.mkdir(outputFolder)
    os.mkdir(agentsFolder)
except:
    pass
pool = []
for i in range(1,21):
    for j in range(1,21):
        pool.append([str(i),str(j)])
for agentsNo in range(4,30,4):
    folder = os.path.join(agentsFolder,str(agentsNo))
    try:
        os.mkdir(folder)
    except:
        pass
    for instance in range(0,20):
        locs = random.sample(pool,agentsNo*2)
        agentsFile = os.path.join(folder,"{}.agents".format(instance))
        with open(agentsFile,"w+") as f:
            f.write("{}\n".format(agentsNo))
            for x in range(0,agentsNo):
                f.write("{},{},\n".format(",".join(locs[x*2]),",".join(locs[x*2+1])))
        for algo in algos:
            for k in range(0,4):
                print("start: agents: {} instance:{} algo: {} k dealy: {}".format(agentsNo,instance,algo,k))
                cmd = ["./Debug/CBSH.exe","-m","{}.map".format(theMap),\
                    "-a",agentsFile,\
                    "-o",os.path.join(outputFolder,'algo={}_agents={}_ins={}_k={}'.format(algo,agentsNo,instance,k)),\
                    "-s", algo,\
                    "-t","300",
                    "--kDelay",str(k)
                        ]
                print(subprocess.list2cmdline(cmd))
                subprocess.run(cmd)
                print("done")

    
    