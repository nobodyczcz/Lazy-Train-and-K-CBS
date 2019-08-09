import subprocess, os

algos=["CBS","CBSH"]
theMap = "./map/0obs-20x20"
output = "./outputs/0obs-20x20map-CBS"
if not os.path.exists(output):
    os.mkdir(output)

for i in range(1,7):
    for k in range(0,5):
        for algo in algos:
            print("start rc {} algo {} k dealy{}".format(i,algo,k))
            cmd = ["./Debug/CBSH.exe","-m","{}.map".format(theMap),\
                "-a","./agent/0obs-20x20map-2agents-{}.agents".format(i),\
                "-o",'{}/algo={}_rc={}_k={}'.format(output,algo,i,k),\
                "-s", algo,\
                "-t","600",
                "--kDelay",str(k)
                    ]
            print(subprocess.list2cmdline(cmd))
            subprocess.run(cmd)
            print("done")
