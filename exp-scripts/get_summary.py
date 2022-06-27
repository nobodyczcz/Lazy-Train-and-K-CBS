import glob, os

def get_summary_old(path):
    folder = path
    csvs = glob.glob(os.path.join(folder,"*"))
    print("found {} csv".format(len(csvs)))
    keywords = ["agents","ins","k","asy","no0","short","corridor","target","flipRec","RM4way","IRM","timeLimit"]
    header = "agents_No,instance,k,asymmetry,no_t0_resolving,short barrier,corridor,target,flipRec,RM4way,I_RM,Time_limit,Runtime,HL_expanded,HL_generated,LL_expanded,LL_generated,agents,solution_cost,idea_cost,algorithm,standard_conflict, rectangle_conflict,corridor2,corridor4,targetRec,ChasingRec,less10,less100,less1000,less10000,less100000,larger100000,num_pairs,num_failed_pairs,valid_train,num_body_conflict,num_target_conflict,num_self_conflict,runtime_mdd"
    with open(os.path.join(folder,"summary.csv"),"w+") as summary:
        count=0
        for csv in csvs:
            if "algo" not in os.path.basename(csv) or ".FailedPairs" in os.path.basename(csv):
                continue
            #print(csv)
            with open(csv,"r") as f:
                try:
                    content = f.readlines()[-1].replace("\n","")
                except:
                    continue
            summary_dict = {}
            filename = os.path.basename(csv)
            info = filename.split("_")
            
            for item in info:
                item_detail = item.split("=")
                summary_dict[item_detail[0]] = item_detail[1]
            
            entry = ""
            for keyword in keywords:
                if keyword not in summary_dict:
                    summary_dict[keyword]="False"
                entry += "{},".format(summary_dict[keyword])
            entry += content
            
            if count == 0:
                elength = len(entry.split(","))
                hlength = len(header.split(","))
                if hlength<elength:
                    header+=","*(elength-hlength)
                    
                summary.write(header+"\n")
            elength = len(entry.split(","))
            hlength = len(header.split(","))
            if (elength < hlength):
                entry += ","*(hlength - elength)
            summary.write(entry+"\n")
            count+=1
        print("Find {} entries".format(count))

def get_summary(path):
    folder = path
    csvs = glob.glob(os.path.join(folder,"*"))
    print("found {} csv".format(len(csvs)))
    keywords = ["agents","ins","k","shrink","ignore-target","lltp-only","corridor","target","parking","algo","ignore-train"]
    header = "agents_No,instance,k,shrink,ignore-target,lltp-only,corridor,target,parking,algo,ignore-train,Runtime,HL_expanded,HL_generated,LL_expanded,LL_generated,agents,solution_cost,cost-dummy,algorithm,standard_conflict,standard_train_conflict,rectangle_sym,corridor_sym,parking_std,target_std,parking_sym,target_sym,train_self_conflict,None,num_llpp,num_lltp,num_cardinal,num_semicardinal,num_noncardinal,num_unkown,valid_train,num_body_conflict,num_target_conflict,num_self_conflict,runtime_mdd"
    with open(os.path.join(folder,"summary.csv"),"w+") as summary:
        count=0
        for csv in csvs:
            if "algo" not in os.path.basename(csv) or ".FailedPairs" in os.path.basename(csv):
                continue
            #print(csv)
            with open(csv,"r") as f:
                try:
                    content = f.readlines()[-1].replace("\n","")
                except:
                    continue
            summary_dict = {}
            filename = os.path.basename(csv)
            info = filename.split("_")
            
            for item in info:
                item_detail = item.split("=")
                summary_dict[item_detail[0]] = item_detail[1]
            
            entry = ""
            for keyword in keywords:
                if keyword not in summary_dict:
                    summary_dict[keyword]="False"
                entry += "{},".format(summary_dict[keyword])
            entry += content
            
            if count == 0:
                elength = len(entry.split(","))
                hlength = len(header.split(","))
                if hlength<elength:
                    header+=","*(elength-hlength)
                    
                summary.write(header+"\n")
            elength = len(entry.split(","))
            hlength = len(header.split(","))
            if (elength < hlength):
                entry += ","*(hlength - elength)
            summary.write(entry+"\n")
            count+=1
        print("Find {} entries".format(count))

def get_summary_m2(path):
    folder = path
    csvs = glob.glob(os.path.join(folder,"*"))
    print("found {} csv".format(len(csvs)))
    keywords = ["agents","ins","k","shrink","ignore-target","lltp-only","corridor","target","parking","algo","ignore-train"]
    header = "agents_No,instance,k,shrink,ignore-target,lltp-only,corridor,target,parking,algo,ignore-train,Runtime,HL_expanded,HL_generated,LL_expanded,LL_generated,agents,solution_cost,cost-dummy,algorithm,standard_conflict,standard_conflict_m2,standard_train_conflict,rectangle_sym,corridor_sym,parking_std,target_std,parking_sym,target_sym,train_self_conflict,None,num_llpp,num_lltp,num_cardinal,num_semicardinal,num_noncardinal,num_unkown,valid_train,num_body_conflict,num_target_conflict,num_self_conflict,runtime_mdd"
    with open(os.path.join(folder,"summary.csv"),"w+") as summary:
        count=0
        for csv in csvs:
            if "algo" not in os.path.basename(csv) or ".FailedPairs" in os.path.basename(csv):
                continue
            #print(csv)
            with open(csv,"r") as f:
                try:
                    content = f.readlines()[-1].replace("\n","")
                except:
                    continue
            summary_dict = {}
            filename = os.path.basename(csv)
            info = filename.split("_")
            
            for item in info:
                item_detail = item.split("=")
                summary_dict[item_detail[0]] = item_detail[1]
            
            entry = ""
            for keyword in keywords:
                if keyword not in summary_dict:
                    summary_dict[keyword]="False"
                entry += "{},".format(summary_dict[keyword])
            entry += content
            
            if count == 0:
                elength = len(entry.split(","))
                hlength = len(header.split(","))
                if hlength<elength:
                    header+=","*(elength-hlength)
                    
                summary.write(header+"\n")
            elength = len(entry.split(","))
            hlength = len(header.split(","))
            if (elength < hlength):
                entry += ","*(hlength - elength)
            summary.write(entry+"\n")
            count+=1
        print("Find {} entries".format(count))
