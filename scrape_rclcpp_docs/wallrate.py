import json
import re
import copy

removeGeneric = re.compile(r"GenericRate(?:<Clock>)?")

generic = {}
with open("jsons/rate.json", "r") as f:
  generic = json.load(f)


print(generic)


wallRate = {}

for key,val in generic.items():
  valCopy = copy.deepcopy(val)
  if (isinstance(valCopy, dict)):
    if (isinstance(valCopy["include"], list)):
      valCopy["include"].append("rclcpp/rclcpp.hpp")
    else:
      valCopy["include"] = "rclcpp/rclcpp.hpp"
    if "template" in valCopy:
      valCopy.pop("template")
  elif isinstance(valCopy, list):
    for elmt in valCopy:
      if isinstance(elmt["include"], list):
        elmt["include"].append("rclcpp/rclcpp.hpp")
      else:
        elmt["include"] = "rclcpp/rclcpp.hpp"
      if "template" in elmt:
        elmt.pop("template")
  wallRate[removeGeneric.sub(r"WallRate", key)] = valCopy
  

print(wallRate)
with open("jsons/wallrate.json", "w") as f:
  f.write(json.dumps(wallRate, indent=4))


tagsDict = {}
with open("tags.json", "r") as f:
  tagsDict = json.load(f)



wr = "wallrate"
t2s = "tag_to_sigs"
s2t = "sig_to_tags"
r = "rate"
tagsDict["tag_to_sigs"][wr] = []

for name,methods in wallRate.items():
  tagsDict[t2s][wr].append(name)
  if name not in tagsDict[s2t]:
    tagsDict[s2t][name] = []
  if isinstance(methods, list):
    for m in methods:
      if "func_type" in m and m["func_type"] == "constructor":
        if "constructor" not in tagsDict[s2t][name]:
          tagsDict[s2t][name].append("constructor")
        if name not in tagsDict[t2s]["constructor"]:
          tagsDict[t2s]["constructor"].append(name)
        break
  elif isinstance(methods, dict):
    if "func_type" in methods and methods["func_type"] == "constructor":
      if "constructor" not in tagsDict[s2t][name]:
        tagsDict[s2t][name].append("constructor")
      if name not in tagsDict[t2s]["constructor"]:
        tagsDict[t2s]["constructor"].append(name)
  if name.find("sleep") > -1 and "sleep" not in tagsDict[s2t][name]:
    tagsDict[s2t][name].append("sleep")
    if name not in tagsDict[t2s]["sleep"]:
      tagsDict[t2s]["sleep"].append(name)
  if wr not in tagsDict[s2t][name]:
    tagsDict[s2t][name].append(wr)
  if r not in tagsDict[s2t][name]:
    tagsDict[s2t][name].append(r)


with open("tags.json", "w") as f:
  f.write(json.dumps(tagsDict, indent=4))
        
