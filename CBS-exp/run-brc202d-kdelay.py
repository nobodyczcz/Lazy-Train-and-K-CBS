import os, random, subprocess

theMap = os.path.abspath('C:/Users/czcz2/Google Drive/MIT/GCITR/CBSH-rect/map/brc202d.map/brc202d')
scen = os.path.abspath('C:/Users/czcz2/Google Drive/MIT/GCITR/CBSH-rect/agent/brc202d.map-scen-even/scen-even/brc202d-even-1.scen')
agentFolder = os.path.abspath('./agent/brc202dSample')
outputFolder = os.path.abspath('./outputs/brc202dSample')
try:
    os.mkdir(agentFolder)
except:
    pass
try:
    os.mkdir(outputFolder)
except:
    pass

algos = ["CBS","CBSH","CBSH-CR","CBSH-R"]

with open(scen, "r") as f:
    agentPool = f.readlines()

agentPool = agentPool[1:]

for agentsNo in range(5,105,5):
    subFolder = os.path.join(agentFolder,str(agentsNo))
    try:
        os.mkdir(subFolder)
    except:
        pass
    for instance in range(0,50):
        agents = random.sample(agentPool,agentsNo)
        outputFile = "{}.agents".format(instance)
        outputPath = os.path.join(subFolder,outputFile)
        if not os.path.exists(outputPath):
            with open(outputPath,"w+") as f:
                f.write(str(agentsNo)+"\n")
                for line in agents:
                    info = line.split("\t")
                    f.write("{},{},{},{},\n".format(info[5],info[4],info[7],info[6]))
        for algo in algos:
            for k in range(0,4):
                resultFile = os.path.join(outputFolder,'algo={}_agents={}_ins={}_k={}'.format(algo,agentsNo,instance,k))
                if os.path.exists(resultFile):
                    print("Pass: agents: {} instance:{} algo: {} k dealy: {}".format(agentsNo,instance,algo,k))
                    continue
                print("start: agents: {} instance:{} algo: {} k dealy: {}".format(agentsNo,instance,algo,k))
                cmd = ["./Debug/CBSH.exe","-m","{}.map".format(theMap),\
                    "-a",outputPath,\
                    "-o",resultFile,\
                    "-s", algo,\
                    "-t","300",
                    "--kDelay",str(k)
                        ]
                print(subprocess.list2cmdline(cmd))
                subprocess.run(cmd)
                print("done")
            
        



