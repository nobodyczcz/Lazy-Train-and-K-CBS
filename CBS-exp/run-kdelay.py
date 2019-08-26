import subprocess, os, time
exe = os.path.abspath("../build/x86-Release/CBSH-rect-cmake/CBS-K.exe") if os.path.exists("../build/x86--Release/CBSH-rect-cmake/CBS-K.exe") else os.path.abspath("../build/linux-x64/CBSH-rect-cmake/CBS-K")
algos=["CBS","CBSH","CBSH-R","CBSH-CR"]
theMap = "./map/0obs-20x20"
output = "./outputs/0obs-20x20map7k"
if not os.path.exists(output):
    os.makedirs(output)
pool=[]
for i in range(2,9):
    for k in range(0,5):
        for algo in algos:
            for asy in [True,False]:
                print("start rc {} algo {} k delay {}".format(i,algo,k))
                cmd = [exe,"-m","{}.map".format(theMap),\
                    "-a","./agent/0obs-20x20map-2agents-{}.agents".format(i),\
                    "-o",'{}/algo={}_rc={}_k={}_asy={}'.format(output,algo,i,k,asy),\
                    "-s", algo,\
                    "-t","300",
                    "--kDelay",str(k),
                        ]
                if asy :
                    cmd.append("--asyConstraint")
                print(subprocess.list2cmdline(cmd))
                if (len(pool)>=6):
                    finish = False
                    while not finish:
                        for p in range(0,len(pool)):
                            if pool[p].poll() is not None:
                                pool.pop(p)
                                finish = True
                                break
                            else:
                                time.sleep(1)
                pool.append(subprocess.Popen(cmd))
