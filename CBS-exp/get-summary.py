import sys, glob, os

folder = sys.argv[1]
csvs = glob.glob(os.path.join(folder,"*"))
print("found {} csv".format(len(csvs)))
with open(os.path.join(folder,"summary.csv"),"w+") as summary:
    summary.write("Rect_square_len,k,Runtime,HL_expanded,HL_generated,LL_expanded,LL_generated,agents,solution_cost,idea_cost,algorithm\n")

    for csv in csvs:
        with open(csv,"r") as f:
            content = f.readline()
        filename = os.path.basename(csv)
        info = filename.split("_")
        print(info)
        rcArea = info[1].split("=")[1]
        k = info[2].split("=")[1]
        
        summary.write("{},{},{}".format(rcArea,k,content))
     
