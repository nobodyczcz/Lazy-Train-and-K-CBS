import pandas as pd
import matplotlib.pyplot as plt
import matplotlib as matplotlib
import numpy as np
import itertools as it
from IPython.display import display, HTML
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D

matplot = matplotlib

def export_legend(legend, filename="legend.png"):
    fig  = legend.figure
    fig.canvas.draw()
    bbox  = legend.get_window_extent().transformed(fig.dpi_scale_trans.inverted())
    fig.savefig(filename, dpi="figure", bbox_inches=bbox)

def pairAnalysisTable(k,results):
    header = ["k","<10","10-100","100-1000","1000-10000",">10000","success rate"]
    data = pd.DataFrame(index = range(0,k+2),columns= header)
    for i in range(0,k+2):
        if i==k+1:
            temp = results
            data.iloc[i,:]["k"] = "Over all"
        else:
            temp = results.loc[results["k"]==i]
            data.iloc[i,:]["k"] = i
        data.iloc[i,:]["<10"] = sum(temp["less10"])
        data.iloc[i,:]["10-100"] = sum(temp["less100"])
        data.iloc[i,:]["100-1000"] = sum(temp["less1000"])
        data.iloc[i,:]["1000-10000"] = sum(temp["less10000"])
        data.iloc[i,:][">10000"] = sum(temp["less100000"])
        total = sum(temp["num_pairs"])
        failed = sum(temp["num_failed_pairs"])
        if total!=0:
            data.iloc[i,:]["success rate"] = (total - failed)/total
    return data

def plotRate(summary,plotTitle="", algo_column = "algorithm", algo_name="", x_label = True, figsize=(4,2.5),k_offset=0.7, k_title=2):

    kGroups = summary.groupby("k")
    for k, kGroup in kGroups:
        marker = it.cycle(('^', '+', 'x', '2', '3','1',"4")) 
        color = it.cycle(('green', 'blue', 'orange', 'red', 'purple','black','cyan','magenta')) 
        algoGroups = kGroup.groupby(algo_column)

        fig = plt.figure(figsize=figsize)
        ax = fig.add_subplot(1,1,1)
        algo_titles = []
        lines = []
        max_agents = 0
        for algo, algoGroup in algoGroups:
            igtrainGroups = algoGroup.groupby("ignore-train")
            for igtrain, igtrainGroup in reversed(list(igtrainGroups)):
                lltpOnlyGroups = igtrainGroup.groupby("lltp-only")
                for lltpOnly, lltpOnlyGroup in reversed(list(lltpOnlyGroups)):
                    shrinkGroups = lltpOnlyGroup.groupby("shrink")
                    for shrink, shrinkGroup in shrinkGroups:
                        igGroups = shrinkGroup.groupby("ignore-target")
                        for ig, igGroup in igGroups:
                            corriGroups = igGroup.groupby("corridor")
                            for corri, corriGroup in corriGroups:
                                parkingGroups = corriGroup.groupby("parking")
                                for parking, parkingGroup in parkingGroups:
                                        targetGroups = parkingGroup.groupby("target")
                                        for target,targetGroup in targetGroups:
                                            title=""
                                            
                                            if algo == "CBS" and lltpOnly:
                                                title = "MT-CBS"
                                            elif algo == "CBS":
                                                title = "LT-CBS"
                                            elif algo == "CBSH" or algo == "CBSH-RM":
                                                title = "LT-CBSH"  
                                            elif algo_name != "":
                                                title = algo_name

    #                                         if ig:
    #                                             title += "v1"

    #                                         if shrink:
    #                                             title += "v2"
    #                                         else:
    #                                             title += "v3"


                                            if algo =="CBSH-RM" and algo_name!="":
                                                 title += "-R"

                                            if corri:
                                                title+="C"
                                            if target:
                                                title+="T"
                                            if parking:
                                                title+="P"
                                                
                                            if (igtrain):
                                                title+="$^{-}$"
                                            x = []
                                            y = []
                                            agentGroups = targetGroup.groupby("agents_No")
                                            for agentNo,agentGroup in agentGroups:

                                                x.append(agentNo)
                                                if (agentNo > max_agents):
                                                    max_agents = agentNo

                                                allRun = len(agentGroup)
#                                                 assert (allRun == 25), str(allRun)
                                                if allRun != 25:
                                                    print(allRun)
                                                    allRun = 25
                                                success = len(agentGroup.loc[(agentGroup["solution_cost"]>0) & (agentGroup["valid_train"] == 1)])
                                                if(not igtrain):
                                                    assert(success ==len(agentGroup.loc[agentGroup["solution_cost"]>0]  ))
                                                successRate = round(success/allRun,2)
                                                y.append(successRate)

                                            lines.append(ax.plot(x,y,marker = next(marker),markersize=10,alpha=0.8,color = next(color), linestyle="--",label=title))
                                            algo_titles.append(title)
        if k==k_title:
            ax.set_title("{} ".format(plotTitle))
        ax.text((ax.get_xlim()[1] - ax.get_xlim()[0])*k_offset + ax.get_xlim()[0],0.9,"$K_{{max}}={}$".format(k),fontsize = 15)
        ax.set_ylim(0, 1.1)
        ax.set_yticks(np.arange(0, 1.25, 0.25))

        if k==8:
            ax.xaxis.set_label_text('No. of Agents',fontsize = 14)

        if x_label:
            ax.yaxis.set_label_text('Success Rate',fontsize = 14)

#         ax.xaxis.set_label_text(plotTitle)
        h, l = ax.get_legend_handles_labels()
        
        figlegend = plt.figure(figsize=(8,1))
        figlegend.legend(h,l,"center",ncol=len(algo_titles),prop={'size': 14})
        figlegend.savefig('legend.png')
        figlegend.show()
        fig.savefig("./figures/{}-{}.png".format(plotTitle,k),bbox_inches='tight')
        fig.show()
        
def plotRateByK(summary,plotTitle="", algo_column = "algorithm", algo_name="", x_label = True):
    line_styles = ["solid","dotted","dashed","dashdot"]
    fig = plt.figure(figsize=(4, 3))
    ax = fig.add_subplot(111)
    agentGroups = summary.groupby("agents_No")
    color = ['blue', 'orange', 'red']
    for agentNo,agentGroup in agentGroups:
#         ls = line_styles.pop()
        marker = it.cycle(('^', '+', 'x', '2', '3','1',"4")) 
        ls = it.cycle(("dashed","solid","dotted","dashdot")) 

        c = color.pop()
#         color = it.cycle(('green', 'blue', 'orange', 'red', 'purple','black','cyan','magenta')) 
        algoGroups = agentGroup.groupby(algo_column)

        algo_titles = []
        lines = []
        for algo, algoGroup in algoGroups:
            igtrainGroups = algoGroup.groupby("ignore-train")
            for igtrain, igtrainGroup in reversed(list(igtrainGroups)):
                lltpOnlyGroups = igtrainGroup.groupby("lltp-only")
                for lltpOnly, lltpOnlyGroup in reversed(list(lltpOnlyGroups)):
                    shrinkGroups = lltpOnlyGroup.groupby("shrink")
                    for shrink, shrinkGroup in shrinkGroups:
                        igGroups = shrinkGroup.groupby("ignore-target")
                        for ig, igGroup in igGroups:
                            corriGroups = igGroup.groupby("corridor")
                            for corri, corriGroup in corriGroups:
                                parkingGroups = corriGroup.groupby("parking")
                                for parking, parkingGroup in parkingGroups:
                                        targetGroups = parkingGroup.groupby("target")
                                        for target,targetGroup in targetGroups:
                                            title=""
                                            
                                            if igtrain and algo !="CBSH-RM": 
                                                title = "K-"+algo
                                            elif igtrain:
                                                title = "K-CBSH"
                                            elif algo == "CBS" and lltpOnly:
                                                title = "MT-CBS"
                                            elif algo == "CBS":
                                                title = "LT-CBS"
                                            elif algo == "CBSH" or algo == "CBSH-RM":
                                                title = "LT-CBSH"  
                                            elif algo_name != "":
                                                title = algo_name

    #                                         if ig:
    #                                             title += "v1"

    #                                         if shrink:
    #                                             title += "v2"
    #                                         else:
    #                                             title += "v3"


                                            if algo =="CBSH-RM" and algo_name!="":
                                                 title += "-R"

                                            if corri:
                                                title+="C"
                                            if target:
                                                title+="T"
                                            if parking:
                                                title+="P"
                                            x = []
                                            y = []
                                            kGroups = targetGroup.groupby("k")
                                            for k, kGroup in kGroups:

                                                x.append(k)

#                                                 allRun = len(kGroup)
#                                                 assert(allRun == 25)
                                                allRun = 25
                                                success = len(kGroup.loc[(kGroup["solution_cost"]>0) & (kGroup["valid_train"] == 1)])
                                                if(not igtrain):
                                                    assert(success ==len(kGroup.loc[kGroup["solution_cost"]>0]  ))
                                                successRate = round(success/allRun,2)
                                                y.append(successRate)

                                            lines.append(ax.plot(x,y,marker = next(marker),markersize=10,alpha=0.8,color = c, linestyle=next(ls),label=title))
                                            algo_titles.append(title)
    ax.set_title("{}".format(plotTitle,agentNo))
    ax.set_ylim(0, 1.1)
#         if k==8:
    ax.xaxis.set_label_text('$k$',fontsize = 14)
#         if x_label:
    ax.yaxis.set_label_text('Success Rate',fontsize = 14)
#         ax.xaxis.set_label_text(plotTitle)
    h, l = ax.get_legend_handles_labels()
#     ax.legend()
    rct = Line2D([5], [0], label='LT-CBSH-RCT', marker='^',linestyle="dashed", color="gray")
    rctp = Line2D([5], [0], label='LT-CBSH-RCTP', marker='+',linestyle="solid", color = "gray")

    patches = [mpatches.Patch(color='red', label='6 Agents'), mpatches.Patch(color='orange', label='8 Agents'),
               mpatches.Patch(color='blue', label='10 Agents'), rct, rctp]

    figlegend = plt.figure(figsize=(2.5,2))
    figlegend.legend(handles=patches,ncol=1,prop={'size': 14}, loc="center")
    figlegend.savefig('legend-long.png')
    fig.savefig("./figures/long-{}-{}.png".format(plotTitle,k),bbox_inches='tight')
    fig.show()
        
def plotTrainRate(summary,plotTitle=""):

    kGroups = summary.groupby("k")
    for k, kGroup in kGroups:
        marker = it.cycle(('^', '+', 'x', '2', '3','1',"4")) 
        color = it.cycle(('green', 'blue', 'orange', 'red', 'purple','black','cyan','magenta')) 
        algoGroups = kGroup.groupby("algorithm")
        plt.figure(figsize=(4, 4))
        for algo, algoGroup in algoGroups:
            if algo == "CBS":
                groupName = "no_t0_resolving"
            else:
                groupName = "asymmetry"
            lGroups = algoGroup.groupby(groupName)
            for name,lGroup in lGroups:
                if groupName == "asymmetry" and name:
                    continue
                
                shortGroups = lGroup.groupby("short barrier")
                for short, shortGroup in shortGroups:
                    corriGroups = shortGroup.groupby("corridor")
                    for corri, corriGroup in corriGroups:
                        I_RMGroups =  corriGroup.groupby("I_RM")
                        for I_RM,I_RMGroup in I_RMGroups:
                            flipGroups = I_RMGroup.groupby("flipRec")
                            for flip, flipGroup in flipGroups:
                                RM4wayGroups=flipGroup.groupby("RM4way")
                                for RM4way, RM4wayGroup in RM4wayGroups:
                                    
                                    targetGroups = RM4wayGroup.groupby("target")
                                    for target,targetGroup in targetGroups:
                                        title=""
                                        if name:
                                            title = "K-{}_{}".format(algo,groupName)
                                        else:
                                            title = "K-{}^{{-}}".format(algo)
                                        if I_RM:
                                            title+="I"
                                        if short:
                                            title += "_shortBarrier"
                                        if corri:
                                            title+="-C"
                                        
                                        if flip:
                                            title+="-F"
#                                         if algo =="CBSH-RM":
#                                             title += "-"+str(RM4way)
                                        if target:
                                            title+="-T"
                                        x = []
                                        successes = []
                                        trainSuccesses = []
                                        agentGroups = targetGroup.groupby("agents_No")
                                        for agentNo,agentGroup in agentGroups:

                                            x.append(agentNo)

                                            allRun = len(agentGroup)
                                            successGroup =agentGroup.loc[agentGroup["solution_cost"]>0] 
                                            success = len(successGroup)
                                            train = len(successGroup.loc[successGroup["valid_train"]==True])
                                            if success ==0:
                                                trainRate = 0
                                            else:
                                                trainRate = round(train/25,2)
                                            successRate = round(success/25,2)
                                            trainSuccesses.append(trainRate)
                                            successes.append(successRate)

                                        plt.plot(x,successes,marker = next(marker),markersize=10,alpha=0.8,color = next(color), linestyle="--",label=title)
                                        if algo!="Train-CBSH" and algo!="Mix-Train-CBSH-RM":
                                            plt.plot(x,trainSuccesses,marker = next(marker),markersize=10,alpha=0.8,color = next(color), linestyle="--",label=title+"-train")

        plt.title("{} K={}".format(plotTitle,k))
        plt.ylim(0, 1.1)
        plt.xlabel('No. of Agents')
        plt.ylabel('Valid Train Plan in Success')
        plt.legend()
        plt.show()
        plt.close()

def plotNodes(summary,plotTitle=""):

    kGroups = summary.groupby("k")
    for k, kGroup in kGroups:
        marker = it.cycle(('^', '+', 'x', '2', '3','1',"4")) 
        color = it.cycle(('green', 'blue', 'orange', 'red', 'purple','black','cyan','magenta')) 
        algoGroups = kGroup.groupby("algorithm")
        plt.figure(figsize=(4, 4))
        for algo, algoGroup in algoGroups:
            if algo == "CBS":
                groupName = "no_t0_resolving"
            else:
                groupName = "asymmetry"
            lGroups = algoGroup.groupby(groupName)
            for name,lGroup in lGroups:
                if groupName == "asymmetry" and name:
                    continue
                
                shortGroups = lGroup.groupby("short barrier")
                for short, shortGroup in shortGroups:
                    corriGroups = shortGroup.groupby("corridor")
                    for corri, corriGroup in corriGroups:
                        I_RMGroups =  corriGroup.groupby("I_RM")
                        for I_RM,I_RMGroup in I_RMGroups:
                            flipGroups = I_RMGroup.groupby("flipRec")
                            for flip, flipGroup in flipGroups:
                                RM4wayGroups=flipGroup.groupby("RM4way")
                                for RM4way, RM4wayGroup in RM4wayGroups:
                                    
                                    targetGroups = RM4wayGroup.groupby("target")
                                    for target,targetGroup in targetGroups:
                                        title=""
                                        if name:
                                            title = "K-{}_{}".format(algo,groupName)
                                        else:
                                            title = "K-{}".format(algo)
                                        if I_RM:
                                            title+="I"
                                        if short:
                                            title += "_shortBarrier"
                                        if corri:
                                            title+="-C"
                                        
                                        if flip:
                                            title+="-F"
#                                         if algo =="CBSH-RM":
#                                             title += "-"+str(RM4way)
                                        if target:
                                            title+="-T"
                                        x = []
                                        y = []
                                        agentGroups = targetGroup.groupby("agents_No")
                                        for agentNo,agentGroup in agentGroups:

                                            x.append(agentNo)

                                            allRun = len(agentGroup)
                                                                                        
                                            y.append(agentGroup["l/h"].mean())

                                        plt.plot(x,y,marker = next(marker),markersize=10,alpha=0.8,color = next(color), linestyle="--",label=title)


        plt.title("{} K={}".format(plotTitle,k))
        plt.xlabel('No. of Agents')
        plt.ylabel('Mean LL_expanded/HL_expanded')
        plt.legend()
        plt.show()
        plt.close()
        
def plotSuccessNodes(summary,plotTitle="",algo_name=""):

    kGroups = summary.groupby("k")
    for k, kGroup in kGroups:
        marker = it.cycle(('^', '+', 'x', '2', '3','1',"4")) 
        color = it.cycle(('green', 'blue', 'orange', 'red', 'purple','black','cyan','magenta')) 
        algoGroups = kGroup.groupby("algorithm")
        plt.figure(figsize=(4, 4))
        for algo, algoGroup in algoGroups:
            if algo == "CBS":
                groupName = "no_t0_resolving"
            else:
                groupName = "asymmetry"
            lGroups = algoGroup.groupby(groupName)
            for name,lGroup in lGroups:
                if groupName == "asymmetry" and name:
                    continue
                
                shortGroups = lGroup.groupby("short barrier")
                for short, shortGroup in shortGroups:
                    corriGroups = shortGroup.groupby("corridor")
                    for corri, corriGroup in corriGroups:
                        I_RMGroups =  corriGroup.groupby("I_RM")
                        for I_RM,I_RMGroup in I_RMGroups:
                            parkingGroups = I_RMGroup.groupby("parking")
                            for parking, parkingGroup in parkingGroups:
                                RM4wayGroups=parkingGroup.groupby("RM4way")
                                for RM4way, RM4wayGroup in RM4wayGroups:
                                    
                                    targetGroups = RM4wayGroup.groupby("target")
                                    for target,targetGroup in targetGroups:
                                        title=""
                                        if algo_name != "":
                                            title = algo_name
                                        
                                        elif name:
                                            title = "K-{}_{}".format(algo,groupName)
                                        else:
                                            title = "K-{}".format(algo)
                                        if I_RM:
                                            title+="I"
                                        if short:
                                            title += "_shortBarrier"
                                            
                                        if algo =="CBSH-RM" and algo_name!="":
                                             title += "-R"
                                        if corri:
                                            title+="C"
                                        if target:
                                            title+="T"                                        
                                        if parking:
                                            title+="P"
#                                         if algo =="CBSH-RM":
#                                             title += "-"+str(RM4way)

                                        x = []
                                        y = []
                                        agentGroups = targetGroup.groupby("agents_No")
                                        for agentNo,agentGroup in agentGroups:

                                            x.append(agentNo)

                                            allRun = len(agentGroup)
                                            successGroup =agentGroup.loc[agentGroup["solution_cost"]>0] 

                                                                                        
                                            y.append(successGroup["HL_expanded"].mean())

                                        plt.plot(x,y,marker = next(marker),markersize=10,alpha=0.8,color = next(color), linestyle="--",label=title)


        plt.title("{} K={}".format(plotTitle,k))
        plt.xlabel('No. of Agents')
        plt.ylabel('Success Instances HL_expanded')
        plt.legend()
        plt.show()
        plt.close()
        
def plotFailedNodes(summary,plotTitle=""):

    kGroups = summary.groupby("k")
    for k, kGroup in kGroups:
        marker = it.cycle(('^', '+', 'x', '2', '3','1',"4")) 
        color = it.cycle(('green', 'blue', 'orange', 'red', 'purple','black','cyan','magenta')) 
        algoGroups = kGroup.groupby("algorithm")
        plt.figure(figsize=(4, 4))
        for algo, algoGroup in algoGroups:
            if algo == "CBS":
                groupName = "no_t0_resolving"
            else:
                groupName = "asymmetry"
            lGroups = algoGroup.groupby(groupName)
            for name,lGroup in lGroups:
                if groupName == "asymmetry" and name:
                    continue
                
                shortGroups = lGroup.groupby("short barrier")
                for short, shortGroup in shortGroups:
                    corriGroups = shortGroup.groupby("corridor")
                    for corri, corriGroup in corriGroups:
                        I_RMGroups =  corriGroup.groupby("I_RM")
                        for I_RM,I_RMGroup in I_RMGroups:
                            flipGroups = I_RMGroup.groupby("flipRec")
                            for flip, flipGroup in flipGroups:
                                RM4wayGroups=flipGroup.groupby("RM4way")
                                for RM4way, RM4wayGroup in RM4wayGroups:
                                    
                                    targetGroups = RM4wayGroup.groupby("target")
                                    for target,targetGroup in targetGroups:
                                        title=""
                                        if name:
                                            title = "K-{}_{}".format(algo,groupName)
                                        else:
                                            title = "K-{}".format(algo)
                                        if I_RM:
                                            title+="I"
                                        if short:
                                            title += "_shortBarrier"
                                        if corri:
                                            title+="-C"
                                        
                                        if flip:
                                            title+="-F"
#                                         if algo =="CBSH-RM":
#                                             title += "-"+str(RM4way)
                                        if target:
                                            title+="-T"
                                        x = []
                                        y = []
                                        agentGroups = targetGroup.groupby("agents_No")
                                        for agentNo,agentGroup in agentGroups:

                                            x.append(agentNo)

                                            allRun = len(agentGroup)
                                            failedGroup =agentGroup.loc[agentGroup["solution_cost"]<0] 

                                                                                        
                                            y.append(failedGroup["HL_expanded"].mean())

                                        plt.plot(x,y,marker = next(marker),markersize=10,alpha=0.8,color = next(color), linestyle="--",label=title)


        plt.title("{} K={}".format(plotTitle,k))
        plt.xlabel('No. of Agents')
        plt.ylabel('Failed Instances HL_expanded')
        plt.legend()
        plt.show()
        plt.close()
        
def plotTimeLimit(summary,plotTitle=""):
    kGroups = summary.groupby("k")
    for k, kGroup in kGroups:
        marker = it.cycle(('^', '+', 'x', '2', '3','1',"4")) 
        color = it.cycle(('green', 'blue', 'orange', 'red', 'purple','black','cyan','magenta')) 
        algoGroups = kGroup.groupby("algorithm")
        plt.figure(figsize=(4, 4))
        for algo, algoGroup in algoGroups:
            if algo == "CBS":
                groupName = "no_t0_resolving"
            else:
                groupName = "asymmetry"
            lGroups = algoGroup.groupby(groupName)
            for name,lGroup in lGroups:
                if groupName == "asymmetry" and name:
                    continue
                
                shortGroups = lGroup.groupby("short barrier")
                for short, shortGroup in shortGroups:
                    corriGroups = shortGroup.groupby("corridor")
                    for corri, corriGroup in corriGroups:
                        I_RMGroups =  corriGroup.groupby("I_RM")
                        for I_RM,I_RMGroup in I_RMGroups:
                            flipGroups = I_RMGroup.groupby("flipRec")
                            for flip, flipGroup in flipGroups:
                                RM4wayGroups=flipGroup.groupby("RM4way")
                                for RM4way, RM4wayGroup in RM4wayGroups:
                                    agentGroups = RM4wayGroup.groupby("agents_No")
                                    for agentNo,agentGroup in agentGroups:
                                        targetGroups = agentGroup.groupby("target")
                                        for target,targetGroup in targetGroups:
                                            title=""
                                            if name:
                                                title = "K-{}_{}_{}".format(algo,groupName,agentNo)
                                            else:
                                                title = "K-{}_{}".format(algo,agentNo)
                                            if I_RM:
                                                title+="I"
                                            if short:
                                                title += "_shortBarrier"
                                            if corri:
                                                title+="-C"

                                            if flip:
                                                title+="-F"
                                            if algo =="CBSH-RM":
                                                title += "-"+str(RM4way)
                                            if target:
                                                title+="-T"

                                            tmGroups = targetGroup.groupby("Time_limit")
                                            x = []
                                            y = []
                                            for tm,tmGroup in tmGroups:                                                
                                                x.append(tm)
                                                allRun = len(tmGroup)
                                                success = len(tmGroup.loc[tmGroup["solution_cost"]>0])
                                                successRate = round(success/allRun,2)
                                                y.append(successRate)

                                            plt.plot(x,y,marker = next(marker),markersize=10,alpha=0.8,color = next(color), linestyle="--",label=title)
        plt.title("{} K={}".format(plotTitle,k))
        plt.ylim(0, 1.1)
        plt.xlabel('Time Limit')
        plt.ylabel('Success Rate')
        plt.legend()
        plt.show()
        plt.close()
        
def plotRecPercent(summarys,labels):
    plt.figure(figsize=(6, 3))
    plt.rc('font', size=14)  
    marker = it.cycle(('^', '+', 'x', '2', '3','1',"4")) 
    color = it.cycle(('green', 'blue', 'orange', 'red', 'purple','black','cyan','magenta')) 
    for i in range(0, len(summarys)):
        kGroups = summarys[i].groupby("k")
        x = []
        y = []
        title = labels[i]
        for k, kGroup in kGroups:
            
            data = kGroup.loc[kGroup["algorithm"] == "CBSH-RM"]
            data = data.loc[data["corridor"] == True]
            data = data.loc[data["target"] == True]
            
            num_stds = data["standard_conflict"].sum()
            num_recs = data["rectangle_conflict"].sum()
            num_corris = data["corridor2"].sum()
            num_targets = data["targetRec"].sum()
            num_all = num_stds+num_recs+num_corris+num_targets
            
            percent = round(num_recs/num_all,3)*100
            x.append(k)
            y.append(percent)
        plt.plot(x,y,marker = next(marker),markersize=10,alpha=0.8,color = next(color), linestyle="--",label=title)
    plt.title("Ratio of Rectangle Conflicts")
    plt.xlabel('k')
    plt.ylabel('Percentage of Rectangle Conflicts / %')
    plt.legend()
    plt.show()
    plt.close()


        
def plotTable(summary):
    level0=[]
    level1=[]
    index=[]
    kGroups = summary.groupby("k")
    for k, kGroup in kGroups: 
        print("---------------------")
        print("K = {}".format(k))
        index.append(k)
        algoGroups = kGroup.groupby("algorithm")
        plt.figure(figsize=(9, 3))
        for algo, algoGroup in algoGroups:
            if algo == "CBS":
                groupName = "no_t0_resolving"
            else:
                groupName = "asymmetry"
            lGroups = algoGroup.groupby(groupName)
            for name,lGroup in lGroups:
                if groupName == "asymmetry" and name:
                    continue
                if name:
                    title = "{}_{}".format(algo,groupName)
                else:
                    title = "{}".format(algo)
                shortGroups = lGroup.groupby("short barrier")
                for short, shortGroup in shortGroups:
                    if short:
                        title += "_shortBarrier"
                    corriGroups = shortGroup.groupby("corridor")
                    for corri, corriGroup in corriGroups:
                        if corri:
                            title+="-C"
                        targetGroups = corriGroup.groupby("target")
                        for target,targetGroup in targetGroups:
                            if target:
                                title+="-T"
                            level0.append("K = {}".format(k))
                            level2.append(title)
                            agentGroups = targetGroup.groupby("agents_No")
                            table = pd.DataFrame(columns=["NO. of agents","Success Rate","HL Node expanded","HL Node generated","LL Node expanded","LL Node generated","avg_runtime","timeout","No of target conflicts"])
                            NoOfAgents = []
                            SucessRateList=[]
                            HLNodeE = []
                            HLNodeG = []
                            LLNodeE = []
                            LLNodeG = []
                            runtime = []
                            timeout = []
                            targetConflicts=[]
                            for agentNo,agentGroup in agentGroups:

                                NoOfAgents.append(agentNo)

                                allRun = len(agentGroup)
                                success = len(agentGroup.loc[agentGroup["solution_cost"]>0])
                                successRate = round(success/allRun,2)
                                SucessRateList.append(successRate)
                                HLNodeE.append(sum(agentGroup["HL_expanded"])/len(agentGroup["HL_expanded"]))
                                HLNodeG.append(sum(agentGroup["HL_generated"])/len(agentGroup["HL_generated"]))
                                LLNodeE.append(sum(agentGroup["LL_expanded"])/len(agentGroup["LL_expanded"]))
                                LLNodeG.append(sum(agentGroup["LL_generated"])/len(agentGroup["LL_generated"]))
                                runtime.append(sum(agentGroup["Runtime"])/len(agentGroup["Runtime"]))
                                timeout.append(len(agentGroup.loc[agentGroup["solution_cost"]<=-1]))
                                targetConflicts.append(sum(agentGroup["target"])/len(agentGroup["target"]))


                            table["NO. of agents"] = NoOfAgents
                            table["Success Rate"] = SucessRateList
                            table["HL Node expanded"] = HLNodeE
                            table["HL Node generated"] = HLNodeG
                            table["LL Node expanded"] = LLNodeE
                            table["LL Node generated"] = LLNodeG
                            table["avg_runtime"] = runtime
                            table["timeout"] = timeout
                            table["No of target conflicts"]=targetConflicts
                            print(title)
                            display(table)
def plotTable2(summary):
    level0=[""]
    level1=["Agents number"]
    indexes=[]
    nodeDic={}
    timeDic={}
    kGroups = summary.groupby("k")
    for k, kGroup in kGroups: 

        ktitle = "K = {}".format(k)
        nodeDic[ktitle]={}
        timeDic[ktitle]={}

        algoGroups = kGroup.groupby("algorithm")
        plt.figure(figsize=(6, 3))
        for algo, algoGroup in algoGroups:
            if algo == "CBS":
                groupName = "no_t0_resolving"
            else:
                groupName = "asymmetry"
            lGroups = algoGroup.groupby(groupName)
            for name,lGroup in lGroups:
                if groupName == "asymmetry" and name:
                    continue
                if name:
                    title = "K-{}_{}".format(algo,groupName)
                else:
                    title = "K-{}".format(algo)
                shortGroups = lGroup.groupby("short barrier")
                for short, shortGroup in shortGroups:
                    if short:
                        title += "_shortBarrier"
                    corriGroups = shortGroup.groupby("corridor")
                    for corri, corriGroup in corriGroups:
                        if corri:
                            title+="-C"
                        targetGroups = corriGroup.groupby("target")
                        for target,targetGroup in targetGroups:
                            if target:
                                title+="-T"
                            rmGroups = targetGroups.groupby("RM4Way")
                            for rm, rmGroup in rmGroups:
                                if algo == "CBSH-RM":
                                    main_title = title+"_"+rm
                                else:
                                    main_title = title
                                
                                agentGroups = rmGroup.groupby("agents_No")
                                table = pd.DataFrame(columns=["NO. of agents","Success Rate","HL Node expanded","HL Node generated","LL Node expanded","LL Node generated","avg_runtime","timeout","No of target conflicts"])
                                NoOfAgents = []
                                SucessRateList=[]
                                HLNodeE = []
                                HLNodeG = []
                                LLNodeE = []
                                LLNodeG = []
                                runtime = []
                                timeout = []
                                targetConflicts=[]
                                indexes=[]
                                for agentNo,agentGroup in agentGroups:
                                    indexes.append(agentNo)

                                    NoOfAgents.append(agentNo)

                                    allRun = len(agentGroup)
                                    success = len(agentGroup.loc[agentGroup["solution_cost"]>0])
                                    successRate = round(success/allRun,2)
                                    SucessRateList.append(successRate)
                                    HLNodeE.append(round(sum(agentGroup["HL_expanded"])/len(agentGroup["HL_expanded"]),2))
                                    HLNodeG.append(sum(agentGroup["HL_generated"])/len(agentGroup["HL_generated"]))
                                    LLNodeE.append(sum(agentGroup["LL_expanded"])/len(agentGroup["LL_expanded"]))
                                    LLNodeG.append(sum(agentGroup["LL_generated"])/len(agentGroup["LL_generated"]))
                                    runtime.append(round(sum(agentGroup["Runtime"])/len(agentGroup["Runtime"]),3))
                                    timeout.append(len(agentGroup.loc[agentGroup["solution_cost"]<=-1]))
                                    targetConflicts.append(sum(agentGroup["target"])/len(agentGroup["target"]))
                                level0.append(ktitle)
                                level1.append(main_title)
                                nodeDic[ktitle][main_title]=HLNodeE
                                timeDic[ktitle][main_title]=runtime

                                table["NO. of agents"] = NoOfAgents
                                table["Success Rate"] = SucessRateList
                                table["HL Node expanded"] = HLNodeE
                                table["HL Node generated"] = HLNodeG
                                table["LL Node expanded"] = LLNodeE
                                table["LL Node generated"] = LLNodeG
                                table["avg_runtime"] = runtime
                                table["timeout"] = timeout
                                table["No of target conflicts"]=targetConflicts
    column1 = [np.array(level0),np.array(level1)]
    table1 = pd.DataFrame(columns = column1)
    table2 = pd.DataFrame(columns = column1)

    table1["","Agents number"]=indexes
    table2["","Agents number"]=indexes

    
    for l0 in level0[1:]:
        for l1 in level1[1:]:
            table1[l0,l1]=nodeDic[l0][l1]
            table2[l0,l1]=timeDic[l0][l1]


    # Set CSS properties for th elements in dataframe
    th_props = [
      ('font-size', '11px'),
      ('text-align', 'center'),
      ('font-weight', 'bold'),
      ('color', '#6d6d6d'),
      ('background-color', '#f7f7f9'),
      ("border-collapse", "collapse"),
      ("border", "1px solid black")
      ]

    # Set CSS properties for td elements in dataframe
    td_props = [
      ('font-size', '11px'),
        ("border-collapse", "collapse"),
      ("border", "1px solid black")
      ]

    # Set table styles
    styles = [
      dict(selector="th", props=th_props),
      dict(selector="td", props=td_props)
      ]
    display(table1.style.set_table_styles(styles).hide_index().highlight_min(color="lightgray", axis=1,subset=["K = 0"]).highlight_min(color="lightgray", axis=1,subset=["K = 1"]).highlight_min(color="lightgray", axis=1,subset=["K = 2"]))
    display(table2.style.set_table_styles(styles).hide_index().highlight_min( color="lightgray",axis=1,subset=["K = 0"]).highlight_min(color="lightgray", axis=1,subset=["K = 1"]).highlight_min(color="lightgray", axis=1,subset=["K = 2"]))

def plotTable3(summary):
    level0=[""]
    level1=["K"]
    indexes=[]
    successDic={}
    timeDic={}
    hNodes={}
    
    kGroups = summary.groupby("agents_No")
    for k, kGroup in kGroups: 

        ktitle = "Agents number = {}".format(k)
        successDic[ktitle]={}
        timeDic[ktitle]={}
        hNodes[ktitle]={}

        algoGroups = kGroup.groupby("algorithm")
        plt.figure(figsize=(9, 3))
        for algo, algoGroup in algoGroups:
            if algo == "CBS":
                groupName = "no_t0_resolving"
            else:
                groupName = "asymmetry"
            lGroups = algoGroup.groupby(groupName)
            for name,lGroup in lGroups:
                if groupName == "asymmetry" and name:
                    continue
                if name:
                    title = "K-{}_{}".format(algo,groupName)
                else:
                    title = "K-{}".format(algo)
                shortGroups = lGroup.groupby("short barrier")
                for short, shortGroup in shortGroups:
                    if short:
                        title += "_shortBarrier"
                    corriGroups = shortGroup.groupby("corridor")
                    for corri, corriGroup in corriGroups:
                        if corri:
                            title+="-C"
                        RM4wayGroups = corriGroup.groupby("RM4way")
                        oldTitle = title
                        for RM4way, RM4wayGroup in RM4wayGroups:
                            if algo=="CBSH-RM":
                                title =oldTitle + "-" + str(RM4way)
                            targetGroups = RM4wayGroup.groupby("target")
                            for target,targetGroup in targetGroups:
                                if target:
                                    title+="-T"
                                agentGroups = targetGroup.groupby("k")
                                table = pd.DataFrame(columns=["NO. of agents","Success Rate","HL Node expanded","HL Node generated","LL Node expanded","LL Node generated","avg_runtime","timeout","No of target conflicts"])
                                NoOfAgents = []
                                SucessRateList=[]
                                HLNodeE = []
                                HLNodeG = []
                                LLNodeE = []
                                LLNodeG = []
                                runtime = []
                                timeout = []
                                targetConflicts=[]
                                indexes=[]
                                for agentNo,agentGroup in agentGroups:
                                    indexes.append(agentNo)

                                    NoOfAgents.append(agentNo)

                                    allRun = len(agentGroup)
                                    success = len(agentGroup.loc[agentGroup["solution_cost"]>0])
                                    successRate = round(success/allRun,2)
                                    SucessRateList.append(successRate)
                                    HLNodeE.append(round(sum(agentGroup["HL_expanded"])/len(agentGroup["HL_expanded"]),2))
                                    HLNodeG.append(sum(agentGroup["HL_generated"])/len(agentGroup["HL_generated"]))
                                    LLNodeE.append(sum(agentGroup["LL_expanded"])/len(agentGroup["LL_expanded"]))
                                    LLNodeG.append(sum(agentGroup["LL_generated"])/len(agentGroup["LL_generated"]))
                                    runtime.append(round(sum(agentGroup["Runtime"])/len(agentGroup["Runtime"]),3))
                                    timeout.append(len(agentGroup.loc[agentGroup["solution_cost"]<=-1]))
                                    targetConflicts.append(sum(agentGroup["target"])/len(agentGroup["target"]))
                                level0.append(ktitle)
                                level1.append(title)
                                successDic[ktitle][title]=SucessRateList
                                timeDic[ktitle][title]=runtime
                                hNodes[ktitle][title]=HLNodeE


                                table["NO. of agents"] = NoOfAgents
                                table["Success Rate"] = SucessRateList
                                table["HL Node expanded"] = HLNodeE
                                table["HL Node generated"] = HLNodeG
                                table["LL Node expanded"] = LLNodeE
                                table["LL Node generated"] = LLNodeG
                                table["avg_runtime"] = runtime
                                table["timeout"] = timeout
                                table["No of target conflicts"]=targetConflicts
    column1 = [np.array(level0),np.array(level1)]
    table1 = pd.DataFrame(columns = column1)
    table2 = pd.DataFrame(columns = column1)
    table3 =pd.DataFrame(columns = column1)

    table1["","K"]=indexes
    table2["","K"]=indexes
    table3["","K"]=indexes

    
    for l0 in level0[1:]:
        for l1 in level1[1:]:
            table1[l0,l1]=successDic[l0][l1]
            table2[l0,l1]=timeDic[l0][l1]
            table3[l0,l1]=hNodes[l0][l1]


    # Set CSS properties for th elements in dataframe
    th_props = [
      ('font-size', '11px'),
      ('text-align', 'center'),
      ('font-weight', 'bold'),
      ('color', '#6d6d6d'),
      ('background-color', '#f7f7f9'),
      ("border-collapse", "collapse"),
      ("border", "1px solid black")
      ]

    # Set CSS properties for td elements in dataframe
    td_props = [
      ('font-size', '11px'),
        ("border-collapse", "collapse"),
      ("border", "1px solid black")
      ]

    # Set table styles
    styles = [
      dict(selector="th", props=th_props),
      dict(selector="td", props=td_props)
      ]
    print("Success Rate")
    display(table1.style.set_table_styles(styles).hide_index().highlight_max(color="lightgray", axis=1,subset=["Agents number = 5"]).highlight_max(color="lightgray", axis=1,subset=["Agents number = 10"]).highlight_max(color="lightgray", axis=1,subset=["Agents number = 15"]))
    print("Runtime")
    display(table2.style.set_table_styles(styles).hide_index().highlight_min( color="lightgray",axis=1,subset=["Agents number = 5"]).highlight_min(color="lightgray", axis=1,subset=["Agents number = 10"]).highlight_min(color="lightgray", axis=1,subset=["Agents number = 15"]))
    print("High Level Nodes")
    display(table3.style.set_table_styles(styles).hide_index().highlight_min( color="lightgray",axis=1,subset=["Agents number = 5"]).highlight_min(color="lightgray", axis=1,subset=["Agents number = 10"]).highlight_min(color="lightgray", axis=1,subset=["Agents number = 15"]))

def plotTable4(summary):
    level0=[""]
    level1=["K"]
    indexes=[]
    successDic={}
    timeDic={}
    kGroups = summary.groupby("agents_No")
    for k, kGroup in kGroups: 

        ktitle = "Agents number = {}".format(k)
        successDic[ktitle]={}
        timeDic[ktitle]={}
        hNodes[ktitle]={}

        algoGroups = kGroup.groupby("algorithm")
        plt.figure(figsize=(9, 3))
        for algo, algoGroup in algoGroups:
            if algo == "CBS":
                groupName = "no_t0_resolving"
            else:
                groupName = "asymmetry"
            lGroups = algoGroup.groupby(groupName)
            for name,lGroup in lGroups:
                if groupName == "asymmetry" and name:
                    continue
                if name:
                    title = "K-{}_{}".format(algo,groupName)
                else:
                    title = "K-{}".format(algo)
                shortGroups = lGroup.groupby("short barrier")
                for short, shortGroup in shortGroups:
                    if short:
                        title += "_shortBarrier"
                    corriGroups = shortGroup.groupby("corridor")
                    for corri, corriGroup in corriGroups:
                        if corri:
                            title+="-C"
                        targetGroups = corriGroup.groupby("target")
                        for target,targetGroup in targetGroups:
                            if target:
                                title+="-T"
                            agentGroups = targetGroup.groupby("k")
                            table = pd.DataFrame(columns=["NO. of agents","Success Rate","HL Node expanded","HL Node generated","LL Node expanded","LL Node generated","avg_runtime","timeout","No of target conflicts"])
                            NoOfAgents = []
                            SucessRateList=[]
                            HLNodeE = []
                            HLNodeG = []
                            LLNodeE = []
                            LLNodeG = []
                            runtime = []
                            timeout = []
                            targetConflicts=[]
                            indexes=[]
                            for agentNo,agentGroup in agentGroups:
                                indexes.append(agentNo)

                                NoOfAgents.append(agentNo)

                                allRun = len(agentGroup)
                                success = len(agentGroup.loc[agentGroup["solution_cost"]>0])
                                successRate = round(success/allRun,2)
                                SucessRateList.append(successRate)
                                HLNodeE.append(round(sum(agentGroup["HL_expanded"])/len(agentGroup["HL_expanded"]),2))
                                HLNodeG.append(sum(agentGroup["HL_generated"])/len(agentGroup["HL_generated"]))
                                LLNodeE.append(sum(agentGroup["LL_expanded"])/len(agentGroup["LL_expanded"]))
                                LLNodeG.append(sum(agentGroup["LL_generated"])/len(agentGroup["LL_generated"]))
                                runtime.append(round(sum(agentGroup["Runtime"])/len(agentGroup["Runtime"]),3))
                                timeout.append(len(agentGroup.loc[agentGroup["solution_cost"]<=-1]))
                                targetConflicts.append(sum(agentGroup["target"])/len(agentGroup["target"]))
                            level0.append(ktitle)
                            level1.append(title)
                            successDic[ktitle][title]=SucessRateList
                            timeDic[ktitle][title]=runtime
                            hNodes[ktitle][title]=HLNodeE

                            table["NO. of agents"] = NoOfAgents
                            table["Success Rate"] = SucessRateList
                            table["HL Node expanded"] = HLNodeE
                            table["HL Node generated"] = HLNodeG
                            table["LL Node expanded"] = LLNodeE
                            table["LL Node generated"] = LLNodeG
                            table["avg_runtime"] = runtime
                            table["timeout"] = timeout
                            table["No of target conflicts"]=targetConflicts
    column1 = [np.array(level0),np.array(level1)]
    table1 = pd.DataFrame(columns = column1)
    table2 = pd.DataFrame(columns = column1)
    table3 =pd.DataFrame(columns = column1)

    table1["","K"]=indexes
    table2["","K"]=indexes
    table3["","K"]=indexes

    
    for l0 in level0[1:]:
        for l1 in level1[1:]:
            table1[l0,l1]=successDic[l0][l1]
            table2[l0,l1]=timeDic[l0][l1]
            table3[l0,l1]=hNodes[l0][l1]


    # Set CSS properties for th elements in dataframe
    th_props = [
      ('font-size', '11px'),
      ('text-align', 'center'),
      ('font-weight', 'bold'),
      ('color', '#6d6d6d'),
      ('background-color', '#f7f7f9'),
      ("border-collapse", "collapse"),
      ("border", "1px solid black")
      ]

    # Set CSS properties for td elements in dataframe
    td_props = [
      ('font-size', '11px'),
        ("border-collapse", "collapse"),
      ("border", "1px solid black")
      ]

    # Set table styles
    styles = [
      dict(selector="th", props=th_props),
      dict(selector="td", props=td_props)
      ]
    display(table1.style.set_table_styles(styles).hide_index().highlight_min(color="lightgray", axis=1,subset=["K = 0"]).highlight_min(color="lightgray", axis=1,subset=["K = 1"]).highlight_min(color="lightgray", axis=1,subset=["K = 2"]))
    display(table2.style.set_table_styles(styles).hide_index().highlight_min( color="lightgray",axis=1,subset=["K = 0"]).highlight_min(color="lightgray", axis=1,subset=["K = 1"]).highlight_min(color="lightgray", axis=1,subset=["K = 2"]))
    display(table3.style.set_table_styles(styles).hide_index().highlight_min( color="lightgray",axis=1,subset=["K = 0"]).highlight_min(color="lightgray", axis=1,subset=["K = 1"]).highlight_min(color="lightgray", axis=1,subset=["K = 2"]))

    


def findDifference(summary,algo1,algo2):
    
    kGroups = summary.groupby("k")
    for k, kGroup in kGroups:
        aGroups = kGroup.groupby("asymmetry")
        for assymmetry,aGroup in aGroups:
            nGroups = aGroup.groupby("no_t0_resolving")
            for no_0,nGroup in nGroups:
                iGroups = nGroup.groupby("instance")
                for instance,iGroup in iGroups:
                    shortGroups = iGroup.groupby("short barrier")
                    for short, shortGroup in shortGroups:
                        agentGroups = shortGroup.groupby("agents_No")
                        for agentNo,agentGroup in agentGroups:
                            if (agentGroup.loc[agentGroup["algorithm"]==algo1].iloc[0]["solution_cost"]>0 and agentGroup.loc[agentGroup["algorithm"]==algo2].iloc[0]["solution_cost"]>0) and (agentGroup.loc[agentGroup["algorithm"]==algo1].iloc[0]["solution_cost"] != agentGroup.loc[agentGroup["algorithm"]==algo2].iloc[0]["solution_cost"]):
                                print(algo1,":")
                                display(agentGroup.loc[agentGroup["algorithm"]==algo1])
                                print(algo2,":")
                                display(agentGroup.loc[agentGroup["algorithm"]==algo2])
def solutionCheck(summary):
    kGroups = summary.groupby("k")
    for k, kGroup in kGroups:
        marker = it.cycle(('^', '+', 'x', '2', '3','1',"4")) 
        color = it.cycle(('green', 'blue', 'orange', 'red', 'purple','black')) 
        corriGroups = kGroup.groupby("corridor")
        for corri, corriGroup in corriGroups:
            targetGroups = corriGroup.groupby("target")
            for target,targetGroup in targetGroups:
                agentGroups = targetGroup.groupby("agents_No")
                for agentNo,agentGroup in agentGroups:
                    insGroups = agentGroup.groupby("instance")
                    for instance, insGroup in insGroups:
                        different=False;
                        solutionCost = None;
                        for index, row in insGroup.iterrows():
                            if row["solution_cost"]<=0:
                                continue
                            if solutionCost ==None:
                                solutionCost = row["solution_cost"]
                            elif solutionCost !=row["solution_cost"]:
                                different=True;
                        if different:
                            group = insGroup.iloc[:,0:22]
                            group = group.sort_values("algorithm")
                            display(group)
                            






        
            
            