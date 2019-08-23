import subprocess, os
exe = os.path.abspath("../build/x86-Release/CBSH-rect-cmake/CBS-K.exe") if os.path.exists("../build/x86--Release/CBSH-rect-cmake/CBS-K.exe") else os.path.abspath("../build/linux-x64/CBSH-rect-cmake/CBS-K")
algos=["CBS","CBSH-CR"]
theMap = "./map/0obs-20x20"
output = "../../outputs"
if not os.path.exists(output):
    os.makedirs(output)

for i in range(2,3):
    for k in range(0,3):
        for algo in algos:
            print("start rc {} algo {} k delay {}".format(i,algo,k))
            cmd = [exe,"-m","{}.map".format(theMap),\
                "-a","./agent/kcar-0obs-20x20map-2agents-{}.agents".format(i),\
                "-o",'{}/kcar-algo={}_rc={}_k={}'.format(output,algo,i,k),\
                "-s", algo,\
                "-t","600",
                "--kDelay",str(k)
                    ]
            print(subprocess.list2cmdline(cmd))
            subprocess.run(cmd)
            print("done")
