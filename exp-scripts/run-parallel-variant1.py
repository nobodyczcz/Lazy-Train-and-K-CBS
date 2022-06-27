import os, random, subprocess,time,glob
import resource

# Maximal virtual memory for subprocesses (in bytes).
MAX_VIRTUAL_MEMORY = 3.5 * 1024 * 1024 * 1024 # 3 GB

def limit_virtual_memory():
    # The tuple below is of the form (soft limit, hard limit). Limit only
    # the soft part so that the limit can be increased later (setting also
    # the hard limit would prevent that).
    # When the limit cannot be changed, setrlimit() raises ValueError.
    resource.setrlimit(resource.RLIMIT_AS, (MAX_VIRTUAL_MEMORY, resource.RLIM_INFINITY))
    
processPool = []
maps = {
        "room-32-32-4":[5,35,5,10],
        "warehouse-10-20-10-2-1":[20,70,10,5],
        "den520d":[20,70,10,5],
        "random-32-32-10":[10,60,10,5],
       }

for mapname,setting in maps.items():
    exe = os.path.abspath("./build-train/CBS-K") #specify the compiled execuatable here.
    theMap = os.path.abspath('./map/{}'.format(mapname))
    scen = os.path.abspath('./agent/{}-even/'.format(mapname))
    outputFolder = os.path.abspath('./2022-may-variant1-m1/{}-even-90'.format(mapname))

    try:
        os.makedirs(outputFolder)
    except:
        pass

    algos = ["CBS","CBSH-RM"]#"CBS","CBSH-CR"
    time_limits = [90]

    count = 0

    for time_limit in time_limits:
        for algo in algos:
            for lltp_only in [True,False]:
                if (lltp_only and algo != "CBS"):
                    continue
                for shrink in [True]:
                    for ignore_target in [False]:
                        if (not shrink) and ignore_target:
                            continue
                        for target in [False, True]:
                            for corri in [False,True]:
                                if(algo == "CBSH-RM" and not (target and corri)):
                                    continue
                                if (algo =="CBS" and (target or corri)):
                                        continue
                                for ignore_train in [False,True]:
                                    for parking in [False]:
                                        if (ignore_train and algo!= "CBSH-RM"):
                                            continue
                                        if (parking and algo != "CBSH-RM"):
                                            continue
                                        if (parking and not target):
                                            continue
                                        if (parking and shrink):
                                            continue
                                        if ((parking or target) and ignore_target):
                                            continue
                                        for instance in range(0,25):
                                            scenFile = "{}-even-{}.scen".format(mapname,instance+1)
                                            scenPath = os.path.join(scen,scenFile)
                                            for agentsNo in range(setting[0],setting[1],setting[2]):

                            #                     if (algo == "CBS" or algo == "CBSH-RM" or algo=="CBSH-R") and asy == True:
                            #                         continue
                            #                     if (algo == "CBSH-CR" or algo == "CBSH-R" or algo == "CBSH-RM") and no0 == True:
                            #                         continue

                                                for k in [2,4,8]:

                                                    out = os.path.join(outputFolder,
                                                                   'algo={}_agents={}_ins={}_k={}_shrink={}_ignore-target={}_corridor={}_target={}_parking={}_lltp-only={}_ignore-train={}'.format(algo+"-DK",agentsNo,instance,k,shrink,ignore_target,corri,target,parking, lltp_only, ignore_train))

#                                                    out = os.path.join(outputFolder,
#                                                                     'algo={}_agents={}_ins={}_k={}_shrink={}_ignore-target={}_corridor={}_target={}_parking={}_lltp-only={}'.format(algo+"-DK",agentsNo,instance,k,shrink,ignore_target,corri,target,parking, lltp_only))
                                                    if os.path.exists(out):
                                                        print("exist")
                                                        continue
                            #                             with open(out,"r") as f:
                            #                                 lines = f.readlines()
                            #                                 if len(lines)!=0:
                            # #                                     print("Pass: agents: {} instance:{} algo: {} k dealy: {}".format(agentsNo,instance,algo,k))
                            #                                     continue
                                                    print("start: agents: {} instance:{} algo: {} k dealy: {} ".format(agentsNo,instance,algo,k))
                                                    cmd = [exe,"-m","{}.map".format(theMap),\
                                                        "-a",str(scenPath),\
                                                        "-o",out,\
                                                        "-s", algo,\
                                                        "-t",str(time_limit),
                                                        "--kDelay",str(k),
                                                        "--diff-k",
                                                        "-k",str(agentsNo)

                                                            ]

                                                    if algo == "CBSH-RM":
                                                        if target:
                                                            cmd+=["--target","true"]
                                                        if corri:
                                                            cmd+=["--corridor","true"]
                                                        if parking:
                                                            cmd+=["--parking", "true"]
                                                    if lltp_only:
                                                        cmd+=["--lltp-only"]
                                                    if shrink:
                                                        cmd+=["--shrink"]
                                                    if ignore_target:
                                                        cmd+=["--ignore-target"]
                                                    if ignore_train:
                                                        cmd+=["--ignore-train"]


                                                    if (len(processPool)>= (setting[3] if algo == "CBSH-RM" else 30)):
                                                        finish = False
                                                        while not finish:
                                                            time.sleep(0.5)
                                                            for p in range(0,len(processPool)):
                                                                if p >= len(processPool):
                                                                    break
                                                                if processPool[p].poll() is not None:
                                                                    processPool.pop(p)
                                                                    p-=1
                                                                    finish = True
                                                    else:
                                                        for p in range(0,len(processPool)):
                                                            if p >= len(processPool):
                                                                break
                                                            if processPool[p].poll() is not None:
                                                                processPool.pop(p)
                                                                p-=1
                                                    print(subprocess.list2cmdline(cmd))

                                                    processPool.append(subprocess.Popen(cmd)) #,preexec_fn=limit_virtual_memory))




time.sleep(90)

