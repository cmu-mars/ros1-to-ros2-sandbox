from bs4 import BeautifulSoup
import bs4
import os
import urllib.request
import re
import json
url = "http://docs.ros2.org/dashing/api/rclcpp/classrclcpp_1_1Node.html"
print(os.path.basename(url))



#print(lines)

def countNumChars(mystr, char):
  count = 0
  for c in mystr:
    if c == char:
      count += 1
  return count

def removeSpacesBeforeAngleBrackets(astring):
  return re.sub(r" >", r">", re.sub(r"< ", r"<", astring))

def putArgsBackTogether(args):
  chooseIndices = []
  ignoreIndices = []
  for i in range(len(args)):
    if i not in ignoreIndices:
      arg = args[i]
      langleCount = countNumChars(arg, "<")
      rangleCount = countNumChars(arg, ">")
      print(langleCount, rangleCount)
      chooseIndices.append(i)
      if langleCount != rangleCount:
        
        oldI = i
        while langleCount != rangleCount and i < len(args) - 1:
          arg = arg + ", " + args[i + 1]
          langleCount = countNumChars(arg, "<")
          rangleCount = countNumChars(arg, ">")
          i += 1
          ignoreIndices.append(i)
        print("New arg: " + arg)
        args[oldI] = re.sub(r"< ", r"<", arg)
      
  return [re.sub(r" >", r">", args[i]) for i in range(len(args)) if i in chooseIndices]
      
def getJsonDict(returnType, args, ref, func_type, include):
  if func_type != "":
    if include != "":
      return {
        "args": args,
        "ref": ref,
        "func_type": func_type,
        "return": returnType,
        "include": include
      }
    return {
      "args": args,
      "return": returnType,
      "func_type": func_type,
      "ref": ref
    }
  return {
    "args": args,
    "ref": ref,
    "return": returnType
  }

def hasClass(tag, classId):
  return "class" in tag.attrs and classId in tag["class"]

def getIncludeString(soup):
  include = ""
  codes = [code for code in soup("code") if code.getText().find("include") > -1]
  if (len(codes)) > 0:
    print("Length of codes array is " + str(len(codes)))
    if (len(codes)) == 1:
      mystr = codes[0].getText().strip()
      include = re.sub(r"#include\s* <([^>]+)>", r"\1", mystr)
      print("The include: " + include)
  return include

def getArrayType(arg):
  if arg.find("[") < 0:
    return ""
  firstIndex = arg.find("[")
  lastIndex = arg.find("]")
  return " " + arg[firstIndex:(lastIndex + 1)]

def hasClassStartingWith(tag, classPrefix):
  if "class" not in tag.attrs:
    return False
  classes = tag["class"]
  for c in classes:
    if c.startswith(classPrefix):
      return True
  return False

def getSoup(url):
  lines = ""
  with urllib.request.urlopen(url) as f:
    lines = f.read().decode("utf-8")
  signatures = {}
  soup = BeautifulSoup(lines, 'lxml')
  return soup

def getNamespace(soup):
  divs = soup("div")
  namespace = ""
  if len(divs) > 0:
    divs = [div for div in divs if hasClass(div, "title")]
    print("len of divs: " + str(len(divs)))
    if (len(divs) == 1):
      mydiv = divs[0]
      print("text: " + mydiv.getText().strip())
      namespace = re.sub(r"([a-zA-Z:_]*) .*", r"\1", mydiv.getText().strip())
      print("namespace: " + namespace)
  return namespace

def getFunctionsTable(soup):
  tables = soup("table")
  print(len(tables))
  declTable = None
  for tab in tables:
    if "class" in tab.attrs:
      if "memberdecls" in tab["class"]:
        print("Found table with class memberdecls")
        rows = tab("tr")
        if len(rows) > 0:
          first = rows[0]
          if first.getText().strip().find("Functions") > -1:
            return tab
  return None

def getClassFunctionsFromRows(rows):
  return [row for row in rows[1:] if hasClassStartingWith(row, "memitem") and not hasClass(row, "inherit")]

def getSignatures(url):
  base_url = url[:(-len(os.path.basename(url)))]
  signatures = {}
  soup = getSoup(url) # BeautifulSoup(lines, 'lxml')
  include = getIncludeString(soup)
  namespace = getNamespace(soup)
  declTable = getFunctionsTable(soup)
  
  if declTable:
    #print(declTable)
    print("declTable does have memberdecls class")
    #for child in declTable.children:
    #  print(child)
    #print(len(declTable.contents))
    rows = declTable("tr")
    #print(len(rows))
    first = rows[0]
    if hasClass(first, "heading"):
      if first.getText().strip().startswith("Public") or first.getText().strip().startswith("Functions"):
        rest = getClassFunctionsFromRows(rows)
        print("len of rest: " + str(len(rest)))

        for row in rest:
          cells = row("td")
          if len(cells) == 2:
            #print("Two cells in this row: " + str(cells))
            cellLeft = cells[0]
            cellRight = cells[1]
            #print(cellLeft)
            #print(cellRight)
            if hasClass(cellLeft, "memItemLeft") and hasClass(cellRight, "memItemRight"):
              returnType = removeSpacesBeforeAngleBrackets(cellLeft.getText().strip())
              if (returnType != "virtual"):
                func_type = ""
                if returnType == "":
                  func_type = "constructor"
                first = cellRight.contents[0]
                if "href" in first.attrs:
                  ref = base_url + first["href"]
                func_name = cellRight.contents[0].getText().strip()
                print("function name: " + func_name)
                otherText = "".join(list(map(lambda x: x if isinstance(x, bs4.NavigableString) else x.getText(), cellRight.contents[1:]))).strip()
                

                lparenIndex = otherText.find("(")
                rparenIndex = otherText.rfind(")")

                # Don't want to include the first paren
                lparenIndex += 1

                # In this case, it was originally -1: nothing found
                if lparenIndex == 0:
                  # Don't actually need to do anything though
                  lparenIndex = 0
                
                if rparenIndex < 0:
                  rparenIndex = len(otherText)

                if lparenIndex <= rparenIndex:
                  otherText = otherText[lparenIndex:rparenIndex]
                  otherText = otherText.strip()
                print("<" + otherText + ">")
                args = putArgsBackTogether(otherText.split(", "))
                print(func_name, args)
                print(args)
                for i in range(len(args)):
                  arg = args[i]
                  spaces = [] #arg.split(" ")
                  argname = "" #spaces[-1]
                  
                  argsplits = arg.split("=")
                  if len(argsplits) > 1:
                    spaces = argsplits[0].split(" ")
                  else:
                    spaces = arg.split(" ")
                  typeName = " ".join(spaces[:-1])
                  argname = spaces[-1]
                  lastIndexOfAmp = argname.rfind("&")
                  if lastIndexOfAmp >= 0:
                    referenceThings = argname[0:(lastIndexOfAmp + 1)]
                    typeName = typeName + referenceThings
                  if len(argsplits) > 1:
                    default_val = argsplits[1]
                    #default_val = re.sub(r'\"', r'\"', default_val)
                    typeName = typeName + " (=" + default_val + ")"
                  args[i] = typeName + getArrayType(arg)
                if (len(args) == 1) and args[0] == "":
                  args = ["void"]
                elif len(args) == 0:
                  args = ["void"]
                func_name = namespace + "::" + func_name
                if func_name not in signatures:
                  signatures[func_name] = []
                  
                sig = getJsonDict(returnType, args, ref, func_type, include)
                signatures[func_name].append(sig)
                
        
      #print("<" + first.getText().strip() + ">")
      #print(first)
    else:
      print("declTable does not have memberdecls class.")


  return signatures



urlDict = {
  "node": "http://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Node.html",
  "publisher": "http://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Publisher.html",
  "subscription": "http://docs.ros2.org/latest/api/rclcpp/classrclcpp_1_1Subscription.html",
  "rclcpp": "http://docs.ros2.org/latest/api/rclcpp/namespacerclcpp.html"
}

mydir = "jsons"
  
if (not os.path.isdir(mydir)):
  os.mkdir(mydir)

for name,url in urlDict.items():
  signatures = getSignatures(url)
  print(signatures)
  with open(os.path.join(mydir, name + ".json"), "w") as f:
    f.write(json.dumps(signatures, indent=4))

