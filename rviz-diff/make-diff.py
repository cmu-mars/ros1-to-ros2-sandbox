from git import Repo
import git
import re

import os
import filecmp

import difflib

# GOTTA USE PATHLIB
import pathlib

repoLoc = os.environ["DIFF_REPO_LOC"] #"/Users/audrey/Personal/School/College/summer2019/rviz"
wkdir = os.getcwd() #"/Users/audrey/Personal/School/College/summer2019/ros1-to-ros2-sandbox/rviz-diff"

repo = Repo(repoLoc)

print(repo.branches)

repo.branches[1].checkout()

print(repo.head.commit)

print(repo.active_branch)

fileName = re.compile(r"^([^/]*/)*(.*)")

kineticCommit = repo.branches[0].commit

diffObj = kineticCommit.diff(repo.head.commit)

#diffs = output.split("diff --git")
#print(len(diffs))

def findChanged(changeType, diffObj):
  changed_diffs = []
  for change in diffObj.iter_change_type(changeType):
    a_base = os.path.basename(change.a_path) if change.a_path else ""
    a_ext = os.path.splitext(change.a_path)[1] if change.a_path else ""
    b_ext = os.path.splitext(change.b_path)[1] if change.b_path else ""
    #print(change.a_path, change.b_path)
    #print(a_ext, b_ext)
    if a_ext != "" or b_ext != "":
      extensions = [a_ext, b_ext]
      #print(extensions)
      if ".h" in extensions or ".c" in extensions or ".hpp" in extensions or ".cpp" in extensions:
        #print("")
        #print(change)
        changed_diffs.append(change)
        #print(added.a_path)
        #print(added.b_path)
        #print("")
  return changed_diffs

def getFilesDict(listOfDiffs):
  diffdict = {}
  for d in listOfDiffs:
    if d.change_type == 'R':
      a_base = os.path.basename(d.a_path)
      b_base = os.path.basename(d.b_path)
      diffdict[a_base] = {
        "original_path": d.a_path,
        "new_path": d.b_path,
        "original_name": a_base,
        "new_name": b_base
      }
    elif d.change_type == 'A':
      b_base = os.path.basename(d.b_path)
      print(b_base)
      if b_base not in diffdict:
        diffdict[b_base] = {
          "original_path": "",
          "new_path": d.b_path,
          "original_name": "",
          "new_name": b_base
        }
        print(d.raw_rename_to, d.raw_rename_from)
      else:
        print("Already found {} in the difference dictionary!".format(b_base))
        print("Other path: {}".format(diffdict[b_base]["new_path"]))
        print("New path: {}".format(d.b_path))
    elif d.change_type == 'D':
      a_base = os.path.basename(d.a_path)
      diffdict[a_base] = {
        "original_path": d.a_path,
        "new_path": "",
        "original_name": a_base,
        "new_name": ""
      }
  return diffdict

added_diffs = findChanged('A', diffObj)

deleted = findChanged('D', diffObj)

renames = findChanged('R', diffObj)

print(len(added_diffs), len(renames), len(deleted))

renamedDict = getFilesDict(renames)

addedDict = getFilesDict(added_diffs)
deletedDict = getFilesDict(deleted)

#print(addedDict)
kineticIndex = git.index.base.IndexFile.from_tree(repo, kineticCommit)


repo.branches[0].checkout()


compares = []

totally_new = []

for a in addedDict:
  modifiedA = re.sub(r"\.hpp", ".h", a)
  print(modifiedA)
  if modifiedA in renamedDict:
    print("Found overlap: {} -> {}".format(renamedDict[modifiedA]["new_path"], addedDict[a]["new_path"]))
  if modifiedA in deletedDict:
    print("Found overlap: {} -> {}".format(deletedDict[modifiedA]["original_path"], addedDict[a]["new_path"]))
    
    #kineticIndex.checkout(paths=deletedDict[modifiedA]["original_path"])
    repoPath = os.path.join(repoLoc, deletedDict[modifiedA]["original_path"])
    destPath = os.path.join(wkdir, deletedDict[modifiedA]["original_path"])

    pathlib.Path(destPath).resolve().parent.mkdir(parents=True, exist_ok=True)
    
    content = ""
    with open(repoPath, "r") as f:
      content = f.read()
    with open(destPath, "w") as f:
      f.write(content)
      f.flush()
    compares.append({
      "ros2_loc": os.path.join(repoLoc, addedDict[a]["new_path"]),
      "ros1_loc": destPath
    })
  if modifiedA not in renamedDict and modifiedA not in deletedDict:
    totally_new.append(addedDict[a]["new_path"])

repo.branches[1].checkout()

differ = difflib.HtmlDiff(tabsize=4)


other_renames = []


for comp in compares:
  ros2 = comp["ros2_loc"]
  ros1 = comp["ros1_loc"]
  if not filecmp.cmp(ros2, ros1, shallow=False):
    lines2 = ["// " + ros2]
    lines1 = ["// " + ros1]
    
    with open(ros2, "r") as f:
      lines2 += f.readlines()

    with open(ros1, "r") as f:
      lines1 += f.readlines()

    with open(ros1 + ".html", "w") as f:
      f.write(differ.make_file(lines1, lines2))
  else:
    other_renames.append((ros1, ros2))

print("")

for new in totally_new:
  print("File '{}' does not have an obvious counterpart".format(new))

for orig,rename in other_renames:
  print("File '{}' renamed to '{}'.".format(orig, rename))
